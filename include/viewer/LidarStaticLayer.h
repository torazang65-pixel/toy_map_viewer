#pragma once

#include "viewer/StaticLayer.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

class LidarStaticLayer : public StaticLayer {
public:
    LidarStaticLayer(const std::string& name, ros::NodeHandle& nh);

    void loadData(const std::string& base_dir, double off_x, double off_y, double off_z) override;
    void clear() override;

private:
    void processLidarFile(const std::string& path, ros::Publisher& pub,
                          double off_x, double off_y, double off_z);

    ros::NodeHandle nh_;
    std::vector<ros::Publisher> lidar_publishers_;
};
