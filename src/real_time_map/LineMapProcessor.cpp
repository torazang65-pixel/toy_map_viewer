#include "real_time_map/LineMapProcessor.h"
#include "common/BinSaver.h"
#include "common/DataTypes.h"
#include <ros/package.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

LineMapProcessor::LineMapProcessor() : nh_("~") {
    loadParameters();
}

void LineMapProcessor::loadParameters() {
    nh_.param<int>("start_index", start_index_, 20000);
    
    // VoxelBuilder 파라미터
    float voxel_size;
    int yaw_voxel_num;
    nh_.param<float>("voxel_size", voxel_size, 0.5f);
    nh_.param<int>("yaw_voxel_num", yaw_voxel_num, 36);

    // RansacLaneGenerator 파라미터
    RansacLaneGenerator::Config ransac_config;
    nh_.param<float>("ransac_search_radius", ransac_config.search_radius, 2.0f);
    nh_.param<float>("ransac_yaw_threshold", ransac_config.yaw_threshold, 10.0f);
    nh_.param<int>("ransac_min_inliers", ransac_config.min_inliers, 5);

    // LaneTracker 파라미터
    LaneTracker::Config tracker_config;
    nh_.param<double>("kf_match_dist", tracker_config.match_distance_threshold, 1.5);
    nh_.param<double>("kf_match_angle", tracker_config.match_angle_threshold, 20.0);
    nh_.param<double>("kf_process_noise", tracker_config.process_noise, 0.01);
    nh_.param<double>("kf_measure_noise", tracker_config.measurement_noise, 0.1);

    voxel_builder_ = std::make_unique<VoxelBuilder>(voxel_size, yaw_voxel_num);
    ransac_lane_generator_ = std::make_unique<RansacLaneGenerator>(ransac_config);
    lane_tracker_ = std::make_unique<LaneTracker>(tracker_config);

    // 경로 설정
    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    std::string converted_folder = "data/issue/converted_bin/";
    
    base_dir_ = pkg_path + "/" + converted_folder + std::to_string(start_index_) + "/";
    batch_dir_ = base_dir_ + "batch/";
    voxel_output_dir_ = base_dir_ + "voxel/";
    lane_output_dir_ = base_dir_ + "lanes/";
    merged_lane_output_dir_ = base_dir_ + "merged_lanes/";

    if (!fs::exists(voxel_output_dir_)) {
        fs::create_directories(voxel_output_dir_);
    }
    if (!fs::exists(lane_output_dir_)) fs::create_directories(lane_output_dir_);
    if (!fs::exists(merged_lane_output_dir_)) fs::create_directories(merged_lane_output_dir_);
}

void LineMapProcessor::run() {
    ROS_INFO(">>> LineMapProcessor Started.");
    
    int batch_idx = 0;
    while (ros::ok()) {
        std::string batch_file = batch_dir_ + "batch_" + std::to_string(batch_idx) + ".bin";
        if (!fs::exists(batch_file)) {
            ROS_INFO("Processed all batches up to %d. Stopping LineMapProcessor.", batch_idx - 1);
            break;
        }

        processBatch(batch_idx);
        batch_idx++;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LineMapProcessor::loadBatchFile(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return cloud;

    uint32_t cluster_num = 0;
    ifs.read(reinterpret_cast<char*>(&cluster_num), 4);

    for (uint32_t c = 0; c < cluster_num; ++c) {
        int32_t id = 0;
        uint32_t p_num = 0;
        ifs.read(reinterpret_cast<char*>(&id), 4);
        ifs.read(reinterpret_cast<char*>(&p_num), 4);

        for (uint32_t i = 0; i < p_num; ++i) {
            pcl::PointXYZI pt;
            float x, y, z, intensity;
            ifs.read(reinterpret_cast<char*>(&x), 4);
            ifs.read(reinterpret_cast<char*>(&y), 4);
            ifs.read(reinterpret_cast<char*>(&z), 4);
            ifs.read(reinterpret_cast<char*>(&intensity), 4);
            
            pt.x = x; pt.y = y; pt.z = z; pt.intensity = intensity;
            cloud->push_back(pt);
        }
    }
    ifs.close();
    return cloud;
}

void LineMapProcessor::processBatch(int batch_index) {
    std::string input_path = batch_dir_ + "batch_" + std::to_string(batch_index) + ".bin";
    std::string output_path = voxel_output_dir_ + "voxel_" + std::to_string(batch_index) + ".bin";
    std::string lane_path = lane_output_dir_ + "lane_" + std::to_string(batch_index) + ".bin";
    std::string merged_lane_path = merged_lane_output_dir_ + "lane_" + std::to_string(batch_index) + ".bin";

    // 1. Load Raw Points
    auto cloud = loadBatchFile(input_path);
    if (cloud->empty()) {
        ROS_WARN("Batch %d is empty.", batch_index);
        return;
    }

    // 2. Voxelization (using VoxelBuilder)
    std::vector<VoxelPoint> voxels = voxel_builder_->build(cloud);
    saveVoxelToBin(output_path, voxels);

    // 3. (Future Step) Lane Generation
    if(!voxels.empty()) {
        std::map<int, Lane> lanes = ransac_lane_generator_->generate(voxels);

        // 디버깅 용
        size_t total_lane_points = 0;
        for (const auto& pair : lanes) {
            total_lane_points += pair.second.points.size();
        }
        ROS_INFO("Batch %d: Generated %lu lanes, Total %lu points.", batch_index, lanes.size(), total_lane_points);

        saveToBin(lane_path, lanes);

        std::map<int, Lane> merged_lanes = lane_tracker_->process(lanes);
        ROS_INFO("Batch %d: KF Merged into %lu lanes.", batch_index, merged_lanes.size());
        saveToBin(merged_lane_path, merged_lanes);

    }

    // 4. Save Result (현재는 Voxel만 저장)

    // ROS_INFO("Batch %d Processed: Raw %lu -> Voxel %lu points", batch_index, cloud->size(), voxels.size());
}