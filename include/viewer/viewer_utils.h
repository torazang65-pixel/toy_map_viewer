#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <filesystem>
#include <string>
#include <vector>

#include <common/data_types.h>
#include <common/io.h>
#include <viewer/viewer_offset.h>

namespace fs = std::filesystem;

namespace realtime_line_generator::viewer {

inline void NormalizeFolder(std::string& folder) {
    if (!folder.empty() && folder.back() != '/') {
        folder.push_back('/');
    }
}

inline bool LoadPointsIfExists(
    const std::string& path,
    std::vector<linemapdraft_builder::data_types::Point>& points) {
    if (!fs::exists(path)) {
        return false;
    }
    return linemapdraft_builder::io::load_points(path, points);
}

inline bool LoadPolylinesIfExists(
    const std::string& path,
    std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines) {
    if (!fs::exists(path)) {
        return false;
    }
    return linemapdraft_builder::io::load_polylines(path, polylines);
}

inline void PublishPointCloud(
    const std::vector<linemapdraft_builder::data_types::Point>& points,
    const OffsetState& offset,
    const std::string& frame_id,
    ros::Publisher& pub) {
    if (points.empty()) {
        return;
    }

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_i(msg, "intensity");

    for (const auto& p : points) {
        *out_x = p.x - offset.x;
        *out_y = p.y - offset.y;
        *out_z = p.z;
        *out_i = p.yaw;
        ++out_x;
        ++out_y;
        ++out_z;
        ++out_i;
    }

    pub.publish(msg);
}

}  // namespace realtime_line_generator::viewer
