#include "viewer/VehicleTrajectoryLayer.h"

#include <geometry_msgs/Point.h>

#include <fstream>
#include <sys/stat.h>

namespace {
    bool fileExists(const std::string& name) {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    }
}

VehicleTrajectoryLayer::VehicleTrajectoryLayer(const std::string& name, ros::NodeHandle& nh)
    : StaticLayer(name), nh_(nh) {}

void VehicleTrajectoryLayer::clear() {
    if (vehicle_trajectory_publisher_) vehicle_trajectory_publisher_.shutdown();
}

void VehicleTrajectoryLayer::processVehicleTrajectory(const std::string& path, ros::Publisher& pub,
                                                      double off_x, double off_y, double off_z) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        ROS_WARN("Vehicle trajectory file not found: %s", path.c_str());
        return;
    }

    uint32_t cluster_num = 0;
    ifs.read(reinterpret_cast<char*>(&cluster_num), 4);

    visualization_msgs::MarkerArray marker_array;
    std::vector<geometry_msgs::Point> trajectory_points;

    for (uint32_t i = 0; i < cluster_num; ++i) {
        int32_t id, layer;
        bool explicit_lane = false;
        uint32_t point_num;

        ifs.read(reinterpret_cast<char*>(&id), 4);
        ifs.read(reinterpret_cast<char*>(&layer), 4);
        ifs.read(reinterpret_cast<char*>(&explicit_lane), sizeof(bool));
        ifs.read(reinterpret_cast<char*>(&point_num), 4);

        for (uint32_t j = 0; j < point_num; ++j) {
            float x, y, z;
            ifs.read(reinterpret_cast<char*>(&x), 4);
            ifs.read(reinterpret_cast<char*>(&y), 4);
            ifs.read(reinterpret_cast<char*>(&z), 4);

            geometry_msgs::Point pt;
            pt.x = x - off_x;
            pt.y = y - off_y;
            pt.z = z - off_z;
            trajectory_points.push_back(pt);

            visualization_msgs::Marker sphere_marker;
            sphere_marker.header.frame_id = "map";
            sphere_marker.header.stamp = ros::Time::now();
            sphere_marker.ns = "vehicle_positions";
            sphere_marker.id = j;
            sphere_marker.type = visualization_msgs::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::Marker::ADD;

            sphere_marker.pose.position = pt;
            sphere_marker.pose.orientation.w = 1.0;

            sphere_marker.scale.x = 1.0;
            sphere_marker.scale.y = 1.0;
            sphere_marker.scale.z = 1.0;

            sphere_marker.color.r = 1.0;
            sphere_marker.color.g = 0.0;
            sphere_marker.color.b = 0.0;
            sphere_marker.color.a = 0.8;

            marker_array.markers.push_back(sphere_marker);
        }
    }

    if (trajectory_points.size() > 1) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "vehicle_trajectory_line";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.3;

        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;

        line_marker.points = trajectory_points;

        marker_array.markers.push_back(line_marker);
    }

    pub.publish(marker_array);
    ROS_INFO("Loaded vehicle trajectory with %lu poses", trajectory_points.size());
}

void VehicleTrajectoryLayer::loadData(const std::string& base_dir, double off_x, double off_y, double off_z) {
    std::string filename = "vehicle_trajectory.bin";
    std::string full_path = base_dir + filename;
    if (!fileExists(full_path)) {
        ROS_INFO("No vehicle trajectory file found");
        return;
    }

    vehicle_trajectory_publisher_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/vehicle_trajectory_viz", 1, true);
    processVehicleTrajectory(full_path, vehicle_trajectory_publisher_, off_x, off_y, off_z);
}
