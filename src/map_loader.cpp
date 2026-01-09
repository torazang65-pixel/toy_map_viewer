#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <nlohmann/json.hpp> // [추가] JSON 헤더

// nlohmann::json을 편하게 쓰기 위한 선언
using json = nlohmann::json;

// 구조체 정의
struct PointXYZI8 {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZI8,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
)

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_loader_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~"); // 프라이빗 파라미터를 읽기 위함

    std::string File_name;
    // 파라미터 "file_name"이 없으면 기본값으로 "20000" 사용
    priv_nh.param<std::string>("file_name", File_name, "20000"); 
    
    ROS_INFO("Working with File_name: %s", File_name.c_str());

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);

    std::string pkg_path = ros::package::getPath("toy_map_viewer");

    // =========================================================
    // [수정] 0.json에서 동적으로 frame_id 읽기
    // =========================================================
    std::string json_path = pkg_path + "/Raw/" + File_name + "/pandar64_0/ego_state/0.json";
    std::ifstream f_json(json_path);
    std::string dynamic_frame_id = "world"; 

    if (f_json.is_open()) {
        json ego_data; // 이제 여기서 에러가 나지 않습니다.
        f_json >> ego_data;
        dynamic_frame_id = ego_data.value("frame_id", "world");
        ROS_INFO("Target frame detected from JSON: %s", dynamic_frame_id.c_str());
    } else {
        ROS_WARN("Could not open %s. Using default frame: world", json_path.c_str());
    }

    // =========================================================
    // .bin 파일 읽기
    // =========================================================
    std::string bin_path = pkg_path + "/data/merged_map/" + File_name + "_final_map.bin";
    pcl::PointCloud<PointXYZI8>::Ptr cloud(new pcl::PointCloud<PointXYZI8>);

    std::ifstream infile(bin_path, std::ios::binary);
    if (!infile) {
        ROS_ERROR("Cannot find bin file: %s", bin_path.c_str());
        return -1;
    }

    while (infile.peek() != EOF) {
        PointXYZI8 pt;
        infile.read((char*)&pt.x, sizeof(float));
        infile.read((char*)&pt.y, sizeof(float));
        infile.read((char*)&pt.z, sizeof(float));
        infile.read((char*)&pt.intensity, sizeof(std::uint8_t));
        cloud->push_back(pt);
    }
    infile.close();

    // ROS 메시지로 변환 및 발행
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = dynamic_frame_id; // 읽어온 frame_id 적용
    msg.header.stamp = ros::Time::now();
    
    pub.publish(msg);
    ROS_INFO("Map Loaded and Published on frame [%s]. Ready to view in RViz!", dynamic_frame_id.c_str());

    ros::spin(); 
    return 0;
}