#include "real_time_map/FrameLoader.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cctype>

FrameLoader::FrameLoader(const ros::NodeHandle& nh) : nh_(nh) {
    nh_.param<std::string>("date", date, "2025-09-26-14-21-28_maxen_v6_2"); 

    std::string pkg_path = ros::package::getPath("toy_map_viewer");

    // 1. 데이터 루트 경로 설정
    base_dir_ = pkg_path + "/lane_change_data_converted/Raw/"+ date + "/frames/";
    std::string zone_info_path = pkg_path + "/lane_change_data/Raw/"+ date + "/zone_info";

    // 2. frame_id.txt 파일에서 문자열 읽기
    std::ifstream id_file(zone_info_path);
    if (id_file.is_open()) {
        if (std::getline(id_file, frame_id_)) {
            // 공백 및 줄바꿈 제거 (GYGI24 뒤의 \n 등 제거) [cite: 3]
            frame_id_.erase(std::remove_if(frame_id_.begin(), frame_id_.end(), 
                            [](unsigned char x){ return std::isspace(x); }), frame_id_.end());
            ROS_INFO("Automatic frame_id detected: %s", frame_id_.c_str());
        }
        id_file.close();
    } else {
        ROS_WARN("Could not find zone_info at: %s. Defaulting to 'map'.", zone_info_path.c_str());
        frame_id_ = "map";
    }
}

std::string FrameLoader::getFrameId() {
    return frame_id_;
}

std::string FrameLoader::getPredBinPath(int frame_index) {
    // 경로 예: .../data/issue/converted_bin/20000/pred_frames/frame_20000.bin
    return base_dir_ + "frame_" + std::to_string(frame_index) + ".bin";
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