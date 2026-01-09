#pragma once

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include "toy_map_viewer/DataTypes.h" // Lane, Point6D 정의 포함

class CoordinateConverter {
public:
    CoordinateConverter();
    ~CoordinateConverter() = default;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string sensor_frame_id_; // 예: "pandar"
    std::string vehicle_frame_id_; // 예: "pcra"

    void run();

private:
    // 단일 프레임 처리 함수
    bool processFrame(int sensor_id, int frame_index, LidarFrame& out_lane);

    // 유틸리티: JSON 및 PCD 경로 생성
    std::string getJsonPath(int sensor_id, int frame_index);
    std::string getPcdPath(int sensor_id, int frame_index);

    // 유틸리티: JSON의 Quaternion을 Eigen Matrix로 변환
    Eigen::Matrix4f getTransformMatrix(double x, double y, double z, 
                                       double q0, double q1, double q2, double q3);

private:
    ros::NodeHandle nh_;
    
    // 파라미터 변수
    std::string base_dir_;
    std::string output_dir_;
    int sensor_id_;
};