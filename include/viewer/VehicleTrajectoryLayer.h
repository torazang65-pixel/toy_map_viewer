#pragma once

#include "viewer/StaticLayer.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

class VehicleTrajectoryLayer : public StaticLayer {
public:
    VehicleTrajectoryLayer(const std::string& name, ros::NodeHandle& nh);

    void loadData(const std::string& base_dir, double off_x, double off_y, double off_z) override;
    void clear() override;

private:
    void processVehicleTrajectory(const std::string& path, ros::Publisher& pub,
                                  double off_x, double off_y, double off_z);

    ros::NodeHandle nh_;
    ros::Publisher vehicle_trajectory_publisher_;
};
