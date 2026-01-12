#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include <nlohmann/json.hpp>

#include "toy_map_viewer/DataTypes.h" // Lane, Point6D 정의 포함

class CoordinateConverter {
public:
    CoordinateConverter();
    ~CoordinateConverter() = default;

    void run();

private:
    // 단일 프레임 처리 함수
    bool processFrame(int sensor_id, int frame_index);
    void saveGlobalMaps();

    // 유틸리티: JSON 및 PCD 경로 생성
    std::string getJsonPath(int frame_index);
    std::string getPcdPath(int frame_index);
    std::string getBinPath(int sensor_id, int frame_index);

private:
    ros::NodeHandle nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_pcd_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_bin_map_;

    // 저장 로직을 공용으로 쓰기 위한 헬퍼 함수
    void saveMapToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map, const std::string& filename);
    
    std::string sensor_frame_id_; // 예: "pandar"
    std::string vehicle_frame_id_; // 예: "pcra"
    std::string target_frame_id_; // 첫 번째 프레임 (기준 좌표계)
    
    // 파라미터 변수
    std::string base_dir_;
    std::string sensor_dir_;
    std::string output_dir_;
    std::string pred_frames_dir_;
    int sensor_id_;
    bool is_first_frame_;
};