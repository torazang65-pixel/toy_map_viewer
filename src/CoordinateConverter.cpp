#include "toy_map_viewer/CoordinateConverter.h"
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "toy_map_viewer/BinSaver.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

CoordinateConverter::CoordinateConverter() 
    : nh_("~"), 
      tf_listener_(tf_buffer_), 
      global_map_(new pcl::PointCloud<pcl::PointXYZI>),
      is_first_frame_(true) 
{
    // 1. 파라미터 로드
    std::string package_name;
    nh_.param<std::string>("package_name", package_name, "toy_map_viewer");
    nh_.param<int>("start_index", sensor_id_, 20000);
    nh_.param<std::string>("sensor_frame", sensor_frame_id_, "pandar64_0");
    nh_.param<std::string>("vehicle_frame", vehicle_frame_id_, "pcra");
    
    // 경로 설정
    std::string pkg_path = ros::package::getPath(package_name);
    base_dir_ = pkg_path + "/data/";
    output_dir_ = pkg_path + "/data/issue/converted_bin/" + std::to_string(sensor_id_) + "/"; // 저장 경로 변경

    // 출력 디렉토리 생성
    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }
}

std::string CoordinateConverter::getJsonPath(int sensor_id, int frame_index) {
    return base_dir_ + "sensor/" + std::to_string(sensor_id) + "/ego_state/" + std::to_string(frame_index) + ".json";
}

std::string CoordinateConverter::getPcdPath(int sensor_id, int frame_index) {
    return base_dir_ + "sensor/" + std::to_string(sensor_id) + "/pcd/" + std::to_string(frame_index) + ".pcd";
}

bool CoordinateConverter::processFrame(int sensor_id, int frame_index) {
    std::string json_path = getJsonPath(sensor_id, frame_index);
    std::string pcd_path = getPcdPath(sensor_id, frame_index);

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

    // 3. PCD 로드 (Intensity 포함)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud_raw) == -1) {
        ROS_WARN("Failed to read PCD: %s", pcd_path.c_str());
        return false;
    }

    // =========================================================
    // [핵심] Map Merge 스타일의 고속 행렬 변환 로직
    // =========================================================
    try {
        // A. Sensor -> Vehicle (T_s2v) : TF 조회
        geometry_msgs::TransformStamped tf_s2v_msg = 
            tf_buffer_.lookupTransform(vehicle_frame_id_, sensor_frame_id_, ros::Time(0), ros::Duration(1.0));
        Eigen::Matrix4f T_s2v = tf2::transformToEigen(tf_s2v_msg).matrix().cast<float>();

        // B. Vehicle -> Source/Map (T_v2s) : JSON 데이터
        Eigen::Translation3f t_v2s(static_cast<float>(ego_data["x"]), 
                                   static_cast<float>(ego_data["y"]), 
                                   static_cast<float>(ego_data["z"]));
        Eigen::Quaternionf r_v2s(static_cast<float>(ego_data["q3"]), // w
                                 static_cast<float>(ego_data["q0"]), // x
                                 static_cast<float>(ego_data["q1"]), // y
                                 static_cast<float>(ego_data["q2"])); // z
        Eigen::Matrix4f T_v2s = (t_v2s * r_v2s).matrix();

        // C. Source -> Target (T_s2t) : TF 조회 (좌표계 보정)
        // 같은 프레임이면 Identity가 반환됨
        Eigen::Matrix4f T_s2t = Eigen::Matrix4f::Identity();
        if (current_frame != target_frame_id_) {
             geometry_msgs::TransformStamped tf_s2t_msg = 
                tf_buffer_.lookupTransform(target_frame_id_, current_frame, ros::Time(0), ros::Duration(1.0));
             T_s2t = tf2::transformToEigen(tf_s2t_msg).matrix().cast<float>();
        }

        // D. 최종 변환 행렬 (T_final = T_s2t * T_v2s * T_s2v)
        Eigen::Matrix4f T_final = T_s2t * T_v2s * T_s2v;

        // 4. 포인트 클라우드 일괄 변환 (고속)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud_raw, *cloud_transformed, T_final);

        // 5. 전역 맵에 누적
        *global_map_ += *cloud_transformed;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF Error at frame %d: %s", frame_index, ex.what());
        return false;
    }

    return true;
}

void CoordinateConverter::saveGlobalMap() {
    if (global_map_->empty()) {
        ROS_WARN("Global map is empty. Nothing to save.");
        return;
    }

    // Voxel Grid Filter (다운샘플링)
    double leaf_size;
    nh_.param<double>("map_leaf_size", leaf_size, 0.1); 

    ROS_INFO("Filtering map with leaf size: %.2f m...", leaf_size);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (leaf_size > 0.0) {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(global_map_);
        voxel_filter.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
        voxel_filter.filter(*filtered_map);
    } else {
        *filtered_map = *global_map_;
    }

    std::string filename = output_dir_ + "lidar_seq_map_0.bin";
    
    saveLidarToBin(filename, filtered_map);
    ROS_INFO(">>> Successfully saved merged map to: %s (Points: %lu)", save_path.c_str(), filtered_map->size());
}

void CoordinateConverter::run() {
    // 1. 파일 검색
    std::string ego_state_dir = base_dir_ + "sensor/" + std::to_string(sensor_id_) + "/ego_state/";
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
    saveGlobalMap();
}