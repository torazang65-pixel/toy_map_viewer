#pragma once

#include <ros/ros.h>
#include <string>
#include <memory> // for std::unique_ptr
#include "real_time_map/VoxelBuilder.h"
#include "real_time_map/RansacLaneGenerator.h"
#include "real_time_map/GreedyLaneGenerator.h"
#include "real_time_map/LaneClusterer.h"

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

private:
    ros::NodeHandle nh_;
    
    // 모듈
    std::unique_ptr<VoxelBuilder> voxel_builder_;
    std::unique_ptr<RansacLaneGenerator> ransac_lane_generator_;
    std::unique_ptr<GreedyLaneGenerator> greedy_lane_generator_;
    std::unique_ptr<LaneClusterer> lane_clusterer_;
    // std::unique_ptr<LaneGenerator> lane_generator_; // 추후 추가 예정

    // 파라미터 및 경로
    int start_index_;
    std::string base_dir_;
    std::string batch_dir_;
    std::string voxel_output_dir_;
    std::string lane_output_dir_;
    std::string merged_lane_output_dir_;
    bool use_greedy_generator_;
};