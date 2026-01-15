#include "toy_map_viewer/CoordinateConverter.h"
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "common/BinSaver.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

struct PointXYZU {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZU,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
)

CoordinateConverter::CoordinateConverter() 
    : nh_("~"), 
      tf_listener_(tf_buffer_), 
      global_pcd_map_(new pcl::PointCloud<pcl::PointXYZI>),
      global_bin_map_(new pcl::PointCloud<pcl::PointXYZI>),
      is_first_frame_(true) 
{
    // 1. 파라미터 로드
    nh_.param<int>("start_index", sensor_id_, 20000);
    nh_.param<std::string>("sensor_frame", sensor_frame_id_, "pandar64_0");
    nh_.param<std::string>("vehicle_frame", vehicle_frame_id_, "pcra");
    nh_.param<std::string>("pred_folder", pred_folder_, "issue_laneline_pred/");
    
    // 경로 설정
    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    base_dir_ = pkg_path + "/data/";
    sensor_dir_ = base_dir_ + "sensor/" + std::to_string(sensor_id_) + "/";
    output_dir_ = pkg_path + "/data/issue/converted_bin/" + std::to_string(sensor_id_) + "/"; // 저장 경로 변경
    pred_frames_dir_ = output_dir_ + "pred_frames/";

    // 출력 디렉토리 생성
    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }
    if(!fs::exists(pred_frames_dir_)) {
        fs::create_directories(pred_frames_dir_);
    }
}

std::string CoordinateConverter::getJsonPath(int frame_index) {
    return sensor_dir_ + "ego_state/" + std::to_string(frame_index) + ".json";
}

std::string CoordinateConverter::getPcdPath(int frame_index) {
    return sensor_dir_ + "pcd/" + std::to_string(frame_index) + ".pcd";
}

std::string CoordinateConverter::getBinPath(int sensor_id, int frame_index) {
    return base_dir_ + pred_folder_ + std::to_string(sensor_id) + "/pandar64_0/" + std::to_string(frame_index) + ".bin";
}

bool CoordinateConverter::processFrame(int sensor_id, int frame_index) {
    std::string json_path = getJsonPath(frame_index);
    std::string pcd_path = getPcdPath(frame_index);
    std::string bin_path = getBinPath(sensor_id, frame_index);

    // 1. JSON 로드
    std::ifstream j_file(json_path);
    if (!j_file.is_open()) return false;
    
    json ego_data;
    try {
        j_file >> ego_data;
    } catch (...) {
        ROS_ERROR("JSON Parse Error at index %d", frame_index);
        return false;
    }

    std::string current_frame = ego_data.value("frame_id", "world");

    // 2. Target Frame 설정 (첫 프레임 기준)
    if (is_first_frame_) {
        target_frame_id_ = current_frame;
        is_first_frame_ = false;
        ROS_INFO("Set Target Frame to: %s", target_frame_id_.c_str());
    }

    // =========================================================
    // [핵심] Map Merge 스타일의 고속 행렬 변환 로직
    // =========================================================
    // 2. 변환 행렬(T_final) 계산
    Eigen::Matrix4f T_final;
    try {
        geometry_msgs::TransformStamped tf_s2v_msg = 
            tf_buffer_.lookupTransform(vehicle_frame_id_, sensor_frame_id_, ros::Time(0), ros::Duration(1.0));
        Eigen::Matrix4f T_s2v = tf2::transformToEigen(tf_s2v_msg).matrix().cast<float>();

        Eigen::Translation3f t_v2s(static_cast<float>(ego_data["x"]), 
                                   static_cast<float>(ego_data["y"]), 
                                   static_cast<float>(ego_data["z"]));
        Eigen::Quaternionf r_v2s(static_cast<float>(ego_data["q3"]), 
                                 static_cast<float>(ego_data["q0"]), 
                                 static_cast<float>(ego_data["q1"]), 
                                 static_cast<float>(ego_data["q2"]));
        Eigen::Matrix4f T_v2s = (t_v2s * r_v2s).matrix();

        Eigen::Matrix4f T_s2t = Eigen::Matrix4f::Identity();
        if (current_frame != target_frame_id_) {
             geometry_msgs::TransformStamped tf_s2t_msg = 
                tf_buffer_.lookupTransform(target_frame_id_, current_frame, ros::Time(0), ros::Duration(1.0));
             T_s2t = tf2::transformToEigen(tf_s2t_msg).matrix().cast<float>();
        }
        T_final = T_s2t * T_v2s * T_s2v;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF Error at frame %d: %s", frame_index, ex.what());
        return false;
    }

    // 3. PCD 데이터 로드 및 변환
    pcl::PointCloud<PointXYZU>::Ptr cloud_pcd_u(new pcl::PointCloud<PointXYZU>);
    if (pcl::io::loadPCDFile<PointXYZU>(pcd_path, *cloud_pcd_u) != -1) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZI>);

        for (const auto& pt_u : cloud_pcd_u->points) {
            pcl::PointXYZI pt_i;
            pt_i.x = pt_u.x;
            pt_i.y = pt_u.y;
            pt_i.z = pt_u.z;

            pt_i.intensity = static_cast<float>(pt_u.intensity)/255.0f;

            cloud_pcd->push_back(pt_i);
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud_pcd, *transformed, T_final);
        *global_pcd_map_ += *transformed;
    }

    // 2. BIN 처리 (cloud_raw를 cloud_bin으로 수정, ifs 선언 확인)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bin(new pcl::PointCloud<pcl::PointXYZI>);
    std::ifstream ifs(bin_path, std::ios::binary);
    if (ifs.is_open()) {
        float buffer[4];
        while (ifs.read(reinterpret_cast<char*>(buffer), sizeof(float) * 4)) {
            pcl::PointXYZI pt;
            pt.x = buffer[0]; pt.y = buffer[1]; pt.z = buffer[2];
            float theta = buffer[3];
            // [voxel_builder.cpp 참고] theta 정규화
            pt.intensity = theta;
            cloud_bin->push_back(pt);
        }
        ifs.close();
        if (!cloud_bin->empty()) {
            // Extract yaw rotation from transformation matrix
            Eigen::Matrix3f rotation = T_final.block<3, 3>(0, 0);
            float yaw_offset = std::atan2(rotation(1, 0), rotation(0, 0));

            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cloud_bin, *transformed, T_final);

            // Apply yaw transformation to theta values
            for (auto& pt : transformed->points) {
                pt.intensity = std::fmod(pt.intensity + yaw_offset + M_PI, 2 * M_PI) - M_PI;
            }

            *global_bin_map_ += *transformed;

            std::string frame_filename = pred_frames_dir_ + "frame_" + std::to_string(frame_index) + ".bin";
            saveLidarToBin(frame_filename, transformed);
        }
    }

    // 4. Store transformed vehicle position (vehicle origin in target frame)
    // The vehicle position in its own frame is at origin (0, 0, 0)
    Eigen::Vector4f vehicle_pos_in_vehicle_frame(0.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4f vehicle_pos_in_target_frame = T_final * vehicle_pos_in_vehicle_frame;

    // Transform the vehicle orientation (quaternion from JSON) to target frame
    // The quaternion in JSON represents vehicle orientation, combine it with transformation
    Eigen::Quaternionf q_transformed(T_final.block<3, 3>(0, 0));

    // Convert quaternion to roll, pitch, yaw
    Eigen::Vector3f euler = q_transformed.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw

    Point6D vehicle_pose;
    vehicle_pose.x = vehicle_pos_in_target_frame(0);
    vehicle_pose.y = vehicle_pos_in_target_frame(1);
    vehicle_pose.z = vehicle_pos_in_target_frame(2);
    vehicle_pose.dx = euler(0);  // roll
    vehicle_pose.dy = euler(1);  // pitch
    vehicle_pose.dz = euler(2);  // yaw

    vehicle_trajectory_.push_back(vehicle_pose);

    return true;
}

void CoordinateConverter::saveMapToFile(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map, const std::string& filename,bool filter_mode) {
    if (map->empty()) return;
    if (filter_mode) {
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(map);
        voxel_filter.setLeafSize(0.3f, 0.3f, 0.3f);
        voxel_filter.filter(*filtered);
    }
    saveLidarToBin(filename, map);// BinSaver 활용
}

void CoordinateConverter::saveVehicleTrajectory() {
    if (vehicle_trajectory_.empty()) {
        ROS_WARN("No vehicle trajectory to save");
        return;
    }

    // Treat vehicle trajectory as a "lane" to reuse saveToBin function
    std::map<int, Lane> trajectory_map;
    Lane trajectory_lane;
    trajectory_lane.id = 9999;  // Special ID for vehicle trajectory
    trajectory_lane.explicit_lane = true;  // Make it visible
    trajectory_lane.points = vehicle_trajectory_;

    trajectory_map[trajectory_lane.id] = trajectory_lane;

    std::string trajectory_filename = output_dir_ + "vehicle_trajectory.bin";
    saveToBin(trajectory_filename, trajectory_map);
    ROS_INFO("Saved Vehicle Trajectory to vehicle_trajectory.bin (%lu poses)", vehicle_trajectory_.size());
}

void CoordinateConverter::saveGlobalMaps() {
    // PCD 결과 저장
    saveMapToFile(global_pcd_map_, output_dir_ + "lidar_seq_0.bin", true);
    ROS_INFO("Saved PCD Global Map to lidar_seq_0.bin");

    // BIN 결과 저장
    saveMapToFile(global_bin_map_, output_dir_ + "lidar_seq_1.bin", false);
    ROS_INFO("Saved BIN Global Map to lidar_seq_1.bin");

    // Vehicle trajectory 저장
    saveVehicleTrajectory();
}

void CoordinateConverter::run() {
    // 1. 파일 검색
    std::string ego_state_dir = sensor_dir_ + "ego_state/";
    if (!fs::exists(ego_state_dir)) {
        ROS_ERROR("Directory not found: %s", ego_state_dir.c_str());
        return;
    }

    std::vector<int> frame_indices;
    for (const auto& entry : fs::directory_iterator(ego_state_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            try {
                frame_indices.push_back(std::stoi(entry.path().stem().string()));
            } catch (...) {}
        }
    }
    std::sort(frame_indices.begin(), frame_indices.end());

    if (frame_indices.empty()) {
        ROS_WARN("No frames found.");
        return;
    }

    ROS_INFO(">>> Starting processing %lu frames...", frame_indices.size());

    // 2. 순차 처리
    int success_count = 0;
    for (int idx : frame_indices) {
        if (processFrame(sensor_id_, idx)) {
            success_count++;
            if (success_count % 50 == 0) ROS_INFO("Processed frame: %d", idx);
        }
    }

    // 3. 결과 저장
    saveGlobalMaps();
}