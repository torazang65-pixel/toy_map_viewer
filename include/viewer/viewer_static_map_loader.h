#pragma once

#include <ros/ros.h>

#include <string>
#include <map>

#include <viewer/viewer_offset.h>

namespace realtime_line_generator::viewer {

class StaticMapLoader {
public:
    StaticMapLoader(ros::NodeHandle& nh, OffsetState& offset);

    void Publish();

private:
    void publishPointsByPolylineId(const std::vector<linemapdraft_builder::data_types::Point>& points,
                                   ros::Publisher& pub,
                                   const std::string& ns,
                                   float scale,
                                   int seq_idx);

    ros::NodeHandle nh_;
    OffsetState& offset_;

    // dynamic publishers for points/lidar sequences
    // Key: topic name (e.g., points_seq_0, lidar_seq_2)
    std::map<std::string, ros::Publisher> points_pubs_;
    std::map<std::string, ros::Publisher> lidar_pubs_;

    std::string output_folder_;
    std::string output_root_;
    std::string converted_root_;
    std::string frame_id_;
    int file_idx_ = 0;
    bool publish_map_points_ = true;
    bool publish_converted_filtered_ = true;
    bool publish_converted_raw_ = true;
};

}  // namespace realtime_line_generator::viewer
