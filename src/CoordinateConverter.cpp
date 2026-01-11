#include "toy_map_viewer/CoordinateConverter.h"
#include "toy_map_viewer/BinSaver.h" // saveToBin 함수 사용
#include <rf_tf_broadcaster/sensor_tf_broadcaster.h>
#include <rf_tf_broadcaster/map_tf_broadcaster.h>
#include <geometry_msgs/Point.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <nlohmann/json.hpp>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

// [추가] 파일 시스템 및 정렬을 위한 헤더
#include <filesystem>
#include <vector>
#include <algorithm>
#include <string>

namespace fs = std::filesystem;
using json = nlohmann::json;

CoordinateConverter::CoordinateConverter() : nh_("~"), tf_listener_(tf_buffer_), base_zone_id_(""), is_base_initialized_(false) {
    // 1. 파라미터 로드
    std::string package_name;
    nh_.param<std::string>("package_name", package_name, "toy_map_viewer");
    nh_.param<int>("start_index", sensor_id_, 1); // 예: sensor/1/
    nh_.param<std::string>("sensor_frame", sensor_frame_id_, "pandar64_0");
    nh_.param<std::string>("vehicle_frame", vehicle_frame_id_, "pcra");
    
    // *참고: start_index, load_count는 이제 사용하지 않고 전체 파일을 스캔합니다.
    
    // 경로 설정 (프로젝트 루트 기준 data 폴더)
    std::string pkg_path = ros::package::getPath(package_name);
    base_dir_ = pkg_path + "/data/";
    output_dir_ = pkg_path + "/data/issue/converted_bin/"; // 출력 경로

    // 출력 디렉토리 생성
    struct stat st = {0};
    if (stat(output_dir_.c_str(), &st) == -1) {
         std::string cmd = "mkdir -p " + output_dir_;
         system(cmd.c_str());
    }
}

std::string CoordinateConverter::getJsonPath(int sensor_id, int frame_index) {
    // data/sensor/{id}/ego_state/{idx}.json
    return base_dir_ + "sensor/" + std::to_string(sensor_id) + "/ego_state/" + std::to_string(frame_index) + ".json";
}

std::string CoordinateConverter::getPcdPath(int sensor_id, int frame_index) {
    // data/sensor/{id}/pcd/{idx}.pcd
    return base_dir_ + "sensor/" + std::to_string(sensor_id) + "/pcd/" + std::to_string(frame_index) + ".pcd";
}

Eigen::Matrix4f CoordinateConverter::getTransformMatrix(double x, double y, double z, 
                                                        double q0, double q1, double q2, double q3) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform(0, 3) = static_cast<float>(x);
    transform(1, 3) = static_cast<float>(y);
    transform(2, 3) = static_cast<float>(z);

    Eigen::Quaternionf q(static_cast<float>(q3), 
                         static_cast<float>(q0), 
                         static_cast<float>(q1), 
                         static_cast<float>(q2));
    
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();

    return transform;
}

geometry_msgs::Point CoordinateConverter::transformPoint(
    const std::string &source_frame,
    const geometry_msgs::Point &input_point,
    const std::string &target_frame
) {
    if(source_frame == target_frame){
        return input_point;
    }

    geometry_msgs::Point output_point;

    try{
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));

        tf2::doTransform(input_point, output_point, tf_stamped);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("CoordinateConverter TF Error (%s -> %s): %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return input_point;
    }
    
    return output_point;
}

bool CoordinateConverter::processFrame(int sensor_id, int frame_index, LidarFrame& out_frame) {
    std::string json_path = getJsonPath(sensor_id, frame_index);
    std::string pcd_path = getPcdPath(sensor_id, frame_index);

    // 1. JSON 파일 로드
    std::ifstream j_file(json_path);
    if (!j_file.is_open()) {
        ROS_WARN("Failed to open JSON: %s", json_path.c_str());
        return false;
    }
    json data;
    try {
        j_file >> data;
    } catch (const std::exception& e) {
        ROS_ERROR("JSON Parse Error: %s", e.what());
        return false;
    }

    std::string zone_id_str = data["frame_id"]; // 곧 구현 예정
    double tx = data["x"];
    double ty = data["y"];
    double tz = data["z"];
    double q0 = data["q0"];
    double q1 = data["q1"];
    double q2 = data["q2"];
    double q3 = data["q3"];

    // frame_id 파싱: "region_zoneIdx" 형식 가정 (예: "SEUL_1")

    if(!is_base_initialized_){
        base_zone_id_ = zone_id_str;
        is_base_initialized_ = true;
        ROS_INFO("Current Zone in : %s", zone_id_str.c_str());
        ROS_INFO("Set Base Zone ID: %s (Frame Index: %d)", base_zone_id_.c_str(), frame_index);
    }

    Eigen::Matrix4f transform = getTransformMatrix(tx, ty, tz, q0, q1, q2, q3);

    // 2. PCD 파일 로드
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud_sensor) == -1) {
        ROS_WARN("Failed to read PCD: %s", pcd_path.c_str());
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vehicle(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_.lookupTransform(vehicle_frame_id_, sensor_frame_id_, ros::Time(0), ros::Duration(1.0));
        pcl_ros::transformPointCloud(*cloud_sensor, *cloud_vehicle, transform_stamped.transform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF Error: %s", ex.what());
        return false;
    }

    // 3. 좌표 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_zone(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_vehicle, *cloud_zone, transform);

    // 4. 결과 저장
    out_frame.id = frame_index;
    out_frame.points.reserve(cloud_zone->points.size());

    for (const auto& pt : cloud_zone->points) {
        geometry_msgs::Point pt_msg;
        pt_msg.x = pt.x;
        pt_msg.y = pt.y;
        pt_msg.z = pt.z;

        geometry_msgs::Point pt_transformed = transformPoint(zone_id_str, pt_msg, base_zone_id_);

        LidarPoint lidar_point;
        lidar_point.x = pt_transformed.x;
        lidar_point.y = pt_transformed.y;
        lidar_point.z = pt_transformed.z;

        /*
        std::strncpy(lidar_point.region, region_str.c_str(), sizeof(lidar_point.region) - 1);
        lidar_point.region[sizeof(lidar_point.region) - 1] = '\0';
        lidar_point.zone_idx = zone_idx;
        */

        out_frame.points.push_back(lidar_point);
    }

    return true;
}

void CoordinateConverter::run() {
    std::map<int, LidarFrame> output_map;
    
    // ego_state 폴더 경로 자동 탐색
    std::string ego_state_dir = base_dir_ + "sensor/" + std::to_string(sensor_id_) + "/ego_state/";

    // 1. 디렉토리 존재 확인
    if (!fs::exists(ego_state_dir)) {
        ROS_ERROR("Directory does not exist: %s", ego_state_dir.c_str());
        return;
    }

    // 2. 디렉토리 내 모든 .json 파일 스캔
    std::vector<int> frame_indices;
    ROS_INFO(">>> Scanning files in directory: %s", ego_state_dir.c_str());

    for (const auto& entry : fs::directory_iterator(ego_state_dir)) {
        // 일반 파일이고 확장자가 .json인 경우만 처리
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            try {
                // 파일명(확장자 제외)을 숫자로 변환 (예: "100.json" -> 100)
                int idx = std::stoi(entry.path().stem().string());
                frame_indices.push_back(idx);
            } catch (...) {
                // 숫자가 아닌 파일명은 무시
                ROS_WARN("Skipping non-integer filename: %s", entry.path().filename().c_str());
            }
        }
    }

    // 3. 인덱스 정렬 (순서대로 처리하기 위함)
    std::sort(frame_indices.begin(), frame_indices.end());

    if (frame_indices.empty()) {
        ROS_WARN("No JSON files found in %s", ego_state_dir.c_str());
        return;
    }

    ROS_INFO(">>> Found %lu frames. Starting batch processing...", frame_indices.size());

    // 4. 모든 프레임 순회하며 변환
    int success_count = 0;
    for (int idx : frame_indices) {
        LidarFrame frame;
        
        if (processFrame(sensor_id_, idx, frame)) {
            output_map[idx] = frame;
            success_count++;
            if (success_count % 100 == 0) ROS_INFO("Processed frame index: %d", idx);
        } else {
            ROS_WARN("Failed to process frame index: %d", idx);
        }
    }

    ROS_INFO(">>> Conversion Completed. Success: %d / %lu", success_count, frame_indices.size());

    if (!output_map.empty()) {
        std::string filename = output_dir_ + "lidar_seq_0.bin";
        saveLidarToBin(filename, output_map);
        ROS_INFO(">>> Converted data saved to: %s", filename.c_str());
    } else {
        ROS_WARN(">>> No data converted. Nothing to save.");
    }
}