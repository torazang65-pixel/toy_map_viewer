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
    nh_.param<int>("batch_size", initial_batch_frames_, 20);
    nh_.param<int>("save_stride", save_stride_, 20);
    
    // VoxelBuilder 파라미터
    float voxel_size;
    int yaw_voxel_num;
    nh_.param<float>("voxel_size", voxel_size, 0.5f);
    nh_.param<int>("yaw_voxel_num", yaw_voxel_num, 36);

    // PCALaneGenerator 파라미터
    PCALaneGenerator::Config pca_config;
    nh_.param<float>("pca_search_radius", pca_config.search_radius, 2.0f);
    nh_.param<float>("pca_yaw_threshold", pca_config.yaw_threshold, 10.0f);
    nh_.param<float>("pca_distance_threshold", pca_config.distance_threshold, 0.2f);
    nh_.param<float>("pca_z_tolerance", pca_config.z_tolerance, 1.0f);
    nh_.param<int>("pca_min_inliers", pca_config.min_inliers, 5);
    nh_.param<int>("pca_min_density", pca_config.min_density, 3);
    nh_.param<float>("pca_max_lane_length", pca_config.max_lane_length, 50.0f);

    // GreedyLaneGenerator 파라미터
    nh_.param<bool>("use_greedy_generator", use_greedy_generator_, false);
    nh_.param<bool>("use_lane_clusterer", use_lane_clusterer_, false);

    GreedyLaneGenerator::Config greedy_config;
    nh_.param<float>("greedy_neighbor_dist", greedy_config.neighbor_dist_thresh, 2.0f);
    nh_.param<float>("greedy_cylinder_width", greedy_config.cylinder_search_width, 1.0f);
    nh_.param<float>("greedy_drop_width", greedy_config.drop_width, 1.0f);
    nh_.param<int>("greedy_min_density", greedy_config.min_density, 5);
    nh_.param<float>("greedy_alpha", greedy_config.alpha, 0.7f);
    nh_.param<float>("greedy_beta", greedy_config.beta, 0.9f);

    // LaneClusterer 파라미터
    LaneClusterer::Config clusterer_config;
    nh_.param<double>("merge_search_radius", clusterer_config.merge_search_radius, 2.5);
    nh_.param<double>("merge_angle_threshold", clusterer_config.merge_angle_threshold, 30.0);
    nh_.param<double>("merge_min_angle_threshold", clusterer_config.merge_min_angle_threshold, 5.0);
    nh_.param<double>("merge_min_dist_for_angle", clusterer_config.merge_min_dist_for_angle, 0.5);
    nh_.param<double>("min_lane_length", clusterer_config.min_lane_length, 3.0);

    // LanePostProcessor 파라미터 로드
    LanePostProcessor::Config pp_config;
    nh_.param<float>("merge_min_dist_th", pp_config.merge_min_dist_th, 1.0f);
    nh_.param<float>("merge_max_dist_th", pp_config.merge_max_dist_th, 10.0f);
    nh_.param<float>("merge_min_angle_th", pp_config.merge_min_angle_th, 0.1f);
    nh_.param<float>("merge_max_angle_th", pp_config.merge_max_angle_th, 0.3f);

    voxel_builder_ = std::make_unique<VoxelBuilder>(voxel_size, yaw_voxel_num);
    voxel_map_ = std::make_unique<AccumulatedVoxelMap>(voxel_size, yaw_voxel_num);
    pca_lane_generator_ = std::make_unique<PCALaneGenerator>(pca_config);
    greedy_lane_generator_ = std::make_unique<GreedyLaneGenerator>(greedy_config);
    lane_clusterer_ = std::make_unique<LaneClusterer>(clusterer_config);
    lane_post_processor_ = std::make_unique<LanePostProcessor>(pp_config);

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
    ROS_INFO(">>> LineMapProcessor Started (Accumulated Voxel Mode).");

    frame_loader_ = std::make_unique<FrameLoader>(nh_);
    int frame_idx = 0;  // 프레임은 0부터 시작
    int loaded_frames = 0;
    int save_idx = 0;  // 저장 인덱스는 별도 관리
    bool initial_phase = true;

    // 요약 정보 수집용 변수
    int total_saves = 0;
    size_t total_generated_lanes = 0;
    size_t total_merged_lanes = 0;

    while (ros::ok()) {
        auto cloud = frame_loader_->loadFrame(frame_idx);
        if (!cloud || cloud->empty()) {
            break;
        }

        voxel_map_->update(cloud);
        loaded_frames++;

        if (initial_phase) {
            if (loaded_frames >= initial_batch_frames_) {
                auto [generated, merged] = processAccumulated(save_idx);
                total_generated_lanes += generated;
                total_merged_lanes += merged;
                total_saves++;
                initial_phase = false;
                save_idx++;
            }
        } else {
            // initial_phase 이후 매 프레임마다 체크
            int frames_since_batch = frame_idx - initial_batch_frames_ + 1;
            if ((save_stride_ > 0) && (frames_since_batch % save_stride_ == 0)) {
                auto [generated, merged] = processAccumulated(save_idx);
                total_generated_lanes += generated;
                total_merged_lanes += merged;
                total_saves++;
                save_idx++;
            }
        }

        frame_idx++;
    }

    // 모든 프로세스 종료 후 요약 정보 출력
    ROS_INFO("========================================");
    ROS_INFO(">>> LineMapProcessor Completed");
    ROS_INFO("  - Total Frames Processed: %d", loaded_frames);
    ROS_INFO("  - Total Saves: %d", total_saves);
    ROS_INFO("  - Total Lanes Generated: %lu", total_generated_lanes);
    ROS_INFO("  - Total Merged Lanes: %lu", total_merged_lanes);
    ROS_INFO("========================================");
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

std::pair<size_t, size_t> LineMapProcessor::processAccumulated(int save_index) {
    std::string output_path = voxel_output_dir_ + "voxel_" + std::to_string(save_index) + ".bin";
    std::string lane_path = lane_output_dir_ + "lane_" + std::to_string(save_index) + ".bin";
    std::string merged_lane_path = merged_lane_output_dir_ + "lane_" + std::to_string(save_index) + ".bin";

    std::vector<VoxelPoint> voxels = voxel_map_->buildVoxels();
    saveVoxelToBin(output_path, voxels);

    size_t generated_lanes_count = 0;
    size_t merged_lanes_count = 0;

    if (!voxels.empty()) {
        std::map<int, Lane> lanes;

        if (use_greedy_generator_) {
            lanes = greedy_lane_generator_->generate(voxels);
        } else {
            lanes = pca_lane_generator_->generate(voxels);
        }

        generated_lanes_count = lanes.size();
        saveToBin(lane_path, lanes);

        std::vector<Lane> clustered_lanes_vec;
        if (use_lane_clusterer_) {
            clustered_lanes_vec = lane_clusterer_->clusterAndSort(lanes);
        } else {
            clustered_lanes_vec = lane_post_processor_->clusterAndSort(lanes);
        }

        std::map<int, Lane> merged_lanes;
        for (const auto& lane : clustered_lanes_vec) {
            merged_lanes[lane.id] = lane;
        }

        merged_lanes_count = merged_lanes.size();
        saveToBin(merged_lane_path, merged_lanes);
    }

    return {generated_lanes_count, merged_lanes_count};
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
        std::map<int, Lane> lanes;

        if (use_greedy_generator_) {
            ROS_INFO("Generating lanes using Greedy Algorithm");
            lanes = greedy_lane_generator_->generate(voxels);
        } else {
            lanes = pca_lane_generator_->generate(voxels);
        }

        // 디버깅 용
        size_t total_lane_points = 0;
        for (const auto& pair : lanes) {
            total_lane_points += pair.second.points.size();
        }
        ROS_INFO("Batch %d: Generated %lu lanes, Total %lu points.", batch_index, lanes.size(), total_lane_points);

        saveToBin(lane_path, lanes);


        std::vector<Lane> clustered_lanes_vec;
        if(use_lane_clusterer_) {
            clustered_lanes_vec = lane_clusterer_->clusterAndSort(lanes);
        } else {
            clustered_lanes_vec = lane_post_processor_->clusterAndSort(lanes);
        }
        
        std::map<int, Lane> merged_lanes;
        for (const auto& lane : clustered_lanes_vec) {
            merged_lanes[lane.id] = lane;
        }
        ROS_INFO("Batch %d: Clustered & Sorted into %lu lanes.", batch_index, merged_lanes.size());
        saveToBin(merged_lane_path, merged_lanes);
    }

    // 4. Save Result (현재는 Voxel만 저장)

    // ROS_INFO("Batch %d Processed: Raw %lu -> Voxel %lu points", batch_index, cloud->size(), voxels.size());
}
