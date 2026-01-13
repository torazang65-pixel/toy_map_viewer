#include "real_time_map/LineMapProcessor.h"
#include "common/BinSaver.h"
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

    // VoxelBuilder 인스턴스 생성
    voxel_builder_ = std::make_unique<VoxelBuilder>(voxel_size, yaw_voxel_num);

    // 경로 설정
    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    std::string converted_folder = "data/issue/converted_bin/";
    
    base_dir_ = pkg_path + "/" + converted_folder + std::to_string(start_index_) + "/";
    batch_dir_ = base_dir_ + "batch/";
    voxel_output_dir_ = base_dir_ + "voxel/";

    if (!fs::exists(voxel_output_dir_)) {
        fs::create_directories(voxel_output_dir_);
    }
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

    // 1. Load Raw Points
    auto cloud = loadBatchFile(input_path);
    if (cloud->empty()) {
        ROS_WARN("Batch %d is empty.", batch_index);
        return;
    }

    // 2. Voxelization (using VoxelBuilder)
    std::vector<VoxelPoint> voxels = voxel_builder_->build(cloud);

    // 3. (Future Step) Lane Generation
    // auto lanes = lane_generator_->generate(voxels);

    // 4. Save Result (현재는 Voxel만 저장)
    saveVoxelToBin(output_path, voxels);

    ROS_INFO("Batch %d Processed: Raw %lu -> Voxel %lu points", 
             batch_index, cloud->size(), voxels.size());
}