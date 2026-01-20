#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <map>
#include <string>

#include "common/DataTypes.h"
#include "LaneUtils.h"

class MapConverterV1 {
public:
    MapConverterV1();
    ~MapConverterV1() = default;

    void run();

private:
    void loadParameters();

private:
    ros::NodeHandle nh_;
    LaneConfig lane_config_;
    std::string package_name_;
    std::string input_folder_name_;
    std::string output_folder_name_;
    std::string base_dir_;
    std::string output_dir_;

    std::map<int, Lane> global_map_;
};
