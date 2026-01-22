#include "real_time_map/FrameLoader.h"
#include <algorithm>
#include <cctype>
#include <iostream>

#include "common/io.h"

FrameLoader::FrameLoader(const ros::NodeHandle& nh) : nh_(nh) {
    nh_.param<std::string>("date", date, "2025-09-26-14-21-28_maxen_v6_2"); 

    std::string pkg_path = ros::package::getPath("realtime_line_generator");

    // 1. 데이터 루트 경로 설정
    base_dir_ = pkg_path + "/data/lane_change_data_converted/Raw/"+ date + "/frames/";
    std::string zone_info_path = pkg_path + "/data/lane_change_data/Raw/"+ date + "/zone_info";

    // 2. zone info
    std::ifstream id_file(zone_info_path);
    if (id_file.is_open()) {
        id_file >> frame_id_;
        ROS_INFO("Automatic frame_id detected: %s", frame_id_.c_str());
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
    std::vector<linemapdraft_builder::data_types::Point> points;
    if (!linemapdraft_builder::io::load_points(bin_path, points)) {
        return nullptr;
    }

    CloudT::Ptr cloud(new CloudT);
    cloud->reserve(points.size());

    for (const auto& p : points) {
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.yaw;
        cloud->push_back(pt);
    }

    return cloud;
}
