#include "toy_map_viewer/CoordinateConverter.h"
#include "toy_map_viewer/BinSaver.h" // saveToBin 함수 사용

#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <nlohmann/json.hpp>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using json = nlohmann::json;

CoordinateConverter::CoordinateConverter() : nh_("~") {
    // 1. 파라미터 로드
    std::string package_name;
    nh_.param<std::string>("package_name", package_name, "toy_map_viewer");
    nh_.param<int>("sensor_id", sensor_id_, 1); // 예: sensor/1/
    nh_.param<int>("start_index", start_index_, 0);
    nh_.param<int>("load_count", load_count_, 10);
    
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
    // 변환 행렬 초기화 (Identity)
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Translation 적용
    transform(0, 3) = static_cast<float>(x);
    transform(1, 3) = static_cast<float>(y);
    transform(2, 3) = static_cast<float>(z);

    // Rotation (Quaternion) 적용
    // Eigen::Quaternionf(w, x, y, z) 순서 주의!
    Eigen::Quaternionf q(static_cast<float>(q3), 
                         static_cast<float>(q0), 
                         static_cast<float>(q1), 
                         static_cast<float>(q2));
    
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();

    return transform;
}

bool CoordinateConverter::processFrame(int sensor_id, int frame_index, Lane& out_lane) {
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

    // 좌표 정보 추출
    double tx = data["x"];
    double ty = data["y"];
    double tz = data["z"];
    double q0 = data["q0"];
    double q1 = data["q1"];
    double q2 = data["q2"];
    double q3 = data["q3"];

    // 변환 행렬 생성 (Sensor -> Zone)
    Eigen::Matrix4f transform = getTransformMatrix(tx, ty, tz, q0, q1, q2, q3);

    // 2. PCD 파일 로드
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
        ROS_WARN("Failed to read PCD: %s", pcd_path.c_str());
        return false;
    }

    // 3. 좌표 변환 (pcl::transformPointCloud)
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // 4. 결과 데이터를 Lane 구조체에 담기 (BinSaver 호환용)
    out_lane.id = frame_index; // ID를 프레임 번호로 설정
    out_lane.type = 0;         // 임의 타입
    out_lane.valid = true;
    out_lane.explicit_lane = true; // 시각화 시 잘 보이게 설정

    for (const auto& pt : transformed_cloud->points) {
        Point6D p6d;
        p6d.x = pt.x;
        p6d.y = pt.y;
        p6d.z = pt.z;
        p6d.dx = 0; p6d.dy = 0; p6d.dz = 0; 
        out_lane.points.push_back(p6d);
    }

    return true;
}

void CoordinateConverter::run() {
    std::map<int, Lane> output_map;

    ROS_INFO(">>> Starting Coordinate Conversion for Sensor %d", sensor_id_);
    ROS_INFO(">>> Range: %d ~ %d (Count: %d)", start_index_, start_index_ + load_count_ - 1, load_count_);

    int success_count = 0;
    for (int i = 0; i < load_count_; ++i) {
        int current_idx = start_index_ + i;
        Lane lane;
        
        if (processFrame(sensor_id_, current_idx, lane)) {
            // 변환된 포인트가 있는 경우에만 맵에 추가
            if (!lane.points.empty()) {
                output_map[current_idx] = lane;
                success_count++;
            }
        }
    }

    ROS_INFO(">>> Conversion Completed. Success: %d / %d", success_count, load_count_);

    if (!output_map.empty()) {
        // 기존 BinSaver 포맷(.bin)으로 저장
        std::string filename = output_dir_ + "points_seq_0.bin"; // 우선 seq_0으로 저장
        saveToBin(filename, output_map);
        ROS_INFO(">>> Saved output to: %s", filename.c_str());
    } else {
        ROS_WARN(">>> No data converted. Nothing to save.");
    }
}