#include "real_time_map/FrameLoader.h"
#include <fstream>
#include <iostream>

FrameLoader::FrameLoader(const ros::NodeHandle& nh) : nh_(nh) {
    // 파라미터 로드
    nh_.param<int>("start_index", sensor_id_, 20000);
    nh_.param<std::string>("converted_folder", input_folder_, "data/issue/converted_bin/"); 

    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    base_dir_ = pkg_path + "/";
}

std::string FrameLoader::getPredBinPath(int frame_index) {
    // 경로 예: .../data/issue/converted_bin/20000/pred_frames/frame_20000.bin
    return base_dir_ + input_folder_ + std::to_string(sensor_id_) + "/pred_frames/frame_" + std::to_string(frame_index) + ".bin";
}

FrameLoader::CloudT::Ptr FrameLoader::loadFrame(int frame_index) {
    std::string bin_path = getPredBinPath(frame_index);
    std::ifstream ifs(bin_path, std::ios::binary);

    if (!ifs.is_open()) {
        // ROS_WARN("Failed to open BIN: %s", bin_path.c_str());
        return nullptr;
    }

    CloudT::Ptr cloud(new CloudT);
    
    uint32_t cluster_num = 0;
    ifs.read(reinterpret_cast<char*>(&cluster_num), sizeof(uint32_t));

    for (uint32_t i = 0; i < cluster_num; ++i) {
        int32_t id;
        uint32_t point_num;
        ifs.read(reinterpret_cast<char*>(&id), sizeof(int32_t));
        ifs.read(reinterpret_cast<char*>(&point_num), sizeof(uint32_t));

        for (uint32_t j = 0; j < point_num; ++j) {
            float x, y, z, intensity;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&y), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&z), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&intensity), sizeof(float)); // float intensity (4bytes)

            pcl::PointXYZI pt;
            pt.x = x; 
            pt.y = y; 
            pt.z = z;
            pt.intensity = intensity;
            cloud->push_back(pt);
        }
    }
    ifs.close();

    return cloud;
}