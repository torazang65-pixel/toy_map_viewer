#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

class FrameLoader {
public:
    using PointT = pcl::PointXYZI;
    using CloudT = pcl::PointCloud<PointT>;

    FrameLoader(const ros::NodeHandle& nh);
    ~FrameLoader() = default;

    CloudT::Ptr loadFrame(int frame_index);

private:
    std::string getPredBinPath(int frame_index);

private:
    ros::NodeHandle nh_;
    
    // 경로 관련 변수
    std::string base_dir_; // 패키지 경로
    std::string input_folder_; // "data/issue/converted_bin/"
    int sensor_id_;
};