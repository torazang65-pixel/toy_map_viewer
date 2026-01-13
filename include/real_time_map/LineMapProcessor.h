#pragma once

#include <ros/ros.h>
#include <string>
#include <memory> // for std::unique_ptr
#include "real_time_map/VoxelBuilder.h"

class LineMapProcessor {
public:
    LineMapProcessor();
    ~LineMapProcessor() = default;

    void run();

private:
    void loadParameters();
    void processBatch(int batch_index);
    
    // 파일 입출력 헬퍼
    pcl::PointCloud<pcl::PointXYZI>::Ptr loadBatchFile(const std::string& path);
    void saveVoxelFile(const std::string& path, const std::vector<VoxelPoint>& voxels);

private:
    ros::NodeHandle nh_;
    
    // 모듈
    std::unique_ptr<VoxelBuilder> voxel_builder_;
    // std::unique_ptr<LaneGenerator> lane_generator_; // 추후 추가 예정

    // 파라미터 및 경로
    int start_index_;
    std::string base_dir_;
    std::string batch_dir_;
    std::string voxel_output_dir_;
};