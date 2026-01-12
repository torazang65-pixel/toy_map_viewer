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
      global_pcd_map_(new pcl::PointCloud<pcl::PointXYZ>),
      global_bin_map_(new pcl::PointCloud<pcl::PointXYZ>),
      is_first_frame_(true) 
{
    // 1. 파라미터 로드
    std::string package_name;
    nh_.param<int>("start_index", sensor_id_, 20000);
    nh_.param<std::string>("sensor_frame", sensor_frame_id_, "pandar64_0");
    nh_.param<std::string>("vehicle_frame", vehicle_frame_id_, "pcra");
    
    // 경로 설정
    std::string pkg_path = ros::package::getPath(package_name);
    base_dir_ = pkg_path + "/data/";
    sensor_dir_ = base_dir_ + "sensor/" + std::to_string(sensor_id_) + "/pandar64_0/";
    output_dir_ = pkg_path + "/data/issue/converted_bin/" + std::to_string(sensor_id_) + "/"; // 저장 경로 변경

    // 출력 디렉토리 생성
    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }
}

std::string CoordinateConverter::getJsonPath(int frame_index) {
    return sensor_dir_ + "ego_state/" + std::to_string(frame_index) + ".json";
}

std::string CoordinateConverter::getPcdPath(int frame_index) {
    return sensor_dir_ + "pcd/" + std::to_string(frame_index) + ".pcd";
}

std::string CoordinateConverter::getBinPath(int sensor_id, int frame_index) {
    return base_dir_ + "issue_laneline_pred/" + std::to_string(sensor_id) + "/pandar64_0/" + std::to_string(frame_index) + ".bin";
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud_pcd) != -1) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcd(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_pcd, *transformed_pcd, T_final);
        *global_pcd_map_ += *transformed_pcd; // PCD 전용 맵에 누적
    }

    // 2. BIN 처리 (cloud_raw를 cloud_bin으로 수정, ifs 선언 확인)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bin(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream ifs(bin_path, std::ios::binary); // ifs 선언
    if (ifs.is_open()) {
        float buffer[4];
        while (ifs.read(reinterpret_cast<char*>(buffer), sizeof(float) * 4)) {
            cloud_bin->push_back(pcl::PointXYZ(buffer[0], buffer[1], buffer[2]));
        }
        ifs.close();

        if (!cloud_bin->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud_bin, *transformed, T_final);
            *global_bin_map_ += *transformed;
        }
    }

    return true;
}

void CoordinateConverter::saveMapToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map, const std::string& filename) {
    if (map->empty()) return;

    double leaf_size;
    nh_.param<double>("map_leaf_size", leaf_size, 0.1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZ>);
    if (leaf_size > 0.0) {
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(map);
        voxel_filter.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
        voxel_filter.filter(*filtered_map);
    } else {
        *filtered_map = *map;
    }

    saveLidarToBin(filename, filtered_map); // BinSaver 활용
}

void CoordinateConverter::saveGlobalMaps() {
    // PCD 결과 저장
    std::string pcd_filename = output_dir_ + "lidar_seq_0.bin";
    saveMapToFile(global_pcd_map_, pcd_filename);
    ROS_INFO("Saved PCD Global Map to lidar_seq_0.bin");

    // BIN 결과 저장
    std::string bin_filename = output_dir_ + "lidar_seq_1.bin";
    saveMapToFile(global_bin_map_, bin_filename);
    ROS_INFO("Saved BIN Global Map to lidar_seq_1.bin");
}

void CoordinateConverter::run() {
    // 1. 파일 검색
    std::string ego_state_dir = sensor_dir_ + "ego_state/"
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