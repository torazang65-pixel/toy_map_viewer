#include <ros/ros.h>
#include <ros/package.h>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <fstream>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>

using json = nlohmann::json;
std::string File_name;

// 사용자 정의 구조체
struct PointXYZI8 {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZI8,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
)


int main(int argc, char** argv) {
    ros::init(argc, argv, "map_merge_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~"); // 프라이빗 파라미터를 읽기 위함

    // 파라미터 "file_name"이 없으면 기본값으로 "20000" 사용
    priv_nh.param<std::string>("file_name", File_name, "20000");
    
    ROS_INFO("Working with File_name: %s", File_name.c_str());

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    pcl::PointCloud<PointXYZI8>::Ptr global_map(new pcl::PointCloud<PointXYZI8>);

    // 0. Target Frame을 저장할 변수 선언
    std::string target_frame = "";
    std::string last_frame_id = "";
    bool frame_changed = false;

    ROS_INFO("Starting dynamic batch processing...");

    int i = 0;
    while (ros::ok()) {
        std::string json_path = pkg_path + "/Raw/" + File_name + "/pandar64_0/ego_state/" + std::to_string(i) + ".json";
        std::string pcd_path = pkg_path + "/Raw/" + File_name + "/pandar64_0/pcd/" + std::to_string(i) + ".pcd";

        std::ifstream f(json_path);
        if (!f.is_open()) {
            ROS_INFO("Reached end of files at index %d.", i);
            break;
        }
        
        json ego_data; 
        f >> ego_data;
        f.close();

        std::string current_frame = ego_data.value("frame_id", "world");

        // 1. 첫 번째 프레임(index 0)에서 target_frame 확정
        if (i == 0) {
            target_frame = current_frame;
            last_frame_id = current_frame;
            ROS_INFO("Target frame set to: %s", target_frame.c_str());
        }

        if (current_frame != last_frame_id) {
            ROS_WARN("!!! Frame ID Changed at index %d: %s -> %s", i, last_frame_id.c_str(), current_frame.c_str());
            frame_changed = true;
            last_frame_id = current_frame; // 업데이트
        }

        pcl::PointCloud<PointXYZI8>::Ptr cloud(new pcl::PointCloud<PointXYZI8>);
        if (pcl::io::loadPCDFile<PointXYZI8>(pcd_path, *cloud) == -1) {
            i++;
            continue;
        }

        try {
            // A. Sensor -> Vehicle (PCRA 기준)
            geometry_msgs::TransformStamped tf_s2v_msg = 
                tfBuffer.lookupTransform("pcra", "pandar64_0", ros::Time(0), ros::Duration(1.0));
            Eigen::Matrix4f T_s2v = tf2::transformToEigen(tf_s2v_msg).matrix().cast<float>();

            // B. Vehicle -> Source Frame (현재 프레임의 좌표계)
            Eigen::Translation3f t_v2s(ego_data["x"], ego_data["y"], ego_data["z"]);
            Eigen::Quaternionf r_v2s(ego_data["q3"], ego_data["q0"], ego_data["q1"], ego_data["q2"]);
            Eigen::Matrix4f T_v2s = (t_v2s * r_v2s).matrix();

            // C. Source Frame -> Target Frame (0번 프레임의 좌표계)
            // source_frame과 target_frame이 같으면(i=0일 때 등) Identity 행렬이 반환됩니다.
            geometry_msgs::TransformStamped tf_s2t_msg = 
                tfBuffer.lookupTransform(target_frame, current_frame, ros::Time(0), ros::Duration(1.0));
            Eigen::Matrix4f T_s2t = tf2::transformToEigen(tf_s2t_msg).matrix().cast<float>();

            // D. 최종 변환 (Sensor -> Vehicle -> Source -> Target)
            Eigen::Matrix4f T_final = T_s2t * T_v2s * T_s2v;

            pcl::PointCloud<PointXYZI8>::Ptr transformed_cloud(new pcl::PointCloud<PointXYZI8>);
            pcl::transformPointCloud(*cloud, *transformed_cloud, T_final);

            *global_map += *transformed_cloud;

            if (i % 50 == 0) {
                ROS_INFO("Accumulating... Frame %d (Source: %s -> Target: %s)", i, current_frame.c_str(), target_frame.c_str());
            }

        } catch (tf2::TransformException &ex) {
            ROS_WARN("Frame %d (Source: %s) failed: %s", i, current_frame.c_str(), ex.what());
        }
        i++;
    }

    pcl::ApproximateVoxelGrid<PointXYZI8> sor;
    sor.setLeafSize(0.1f, 0.1f, 0.1f); //필터
    sor.setInputCloud(global_map);

    pcl::PointCloud<PointXYZI8>::Ptr filtered_map(new pcl::PointCloud<PointXYZI8>);
    sor.filter(*filtered_map);

    global_map->clear();

    // .bin 파일로 저장 (x, y, z, intensity 순서)
    std::string save_path = pkg_path + "/data/merged_map/" + File_name+ "_final_map.bin";
    std::ofstream outfile(save_path, std::ios::binary);
    
    for (const auto& pt : *filtered_map) {
        outfile.write((char*)&pt.x, sizeof(float));
        outfile.write((char*)&pt.y, sizeof(float));
        outfile.write((char*)&pt.z, sizeof(float));
        outfile.write((char*)&pt.intensity, sizeof(std::uint8_t));
    }
    outfile.close();

    ROS_INFO("Final Map saved to %s (Points: %lu)", save_path.c_str(), filtered_map->size());
    return 0;
}