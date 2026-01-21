#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <map>
#include <string>

#include <common/data_types.h>
#include "LaneUtils.h"

class MapConverterV2 {
public:
    MapConverterV2();
    ~MapConverterV2() = default;

    void run();

private:
    void loadParameters();
    void processMap(const std::string& input_subdir, const std::string& output_filename);

private:
    ros::NodeHandle nh_;
    LaneConfig lane_config_;
    std::string package_name_;
    std::string base_input_dir_;
    std::string output_root_dir_;

    std::map<int, linemapdraft_builder::data_types::Lane> global_map_;
};
