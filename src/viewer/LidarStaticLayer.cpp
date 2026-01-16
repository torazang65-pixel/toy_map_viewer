#include "viewer/LidarStaticLayer.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <sys/stat.h>
#include <cmath>

namespace {
    bool fileExists(const std::string& name) {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    }
}

LidarStaticLayer::LidarStaticLayer(const std::string& name, ros::NodeHandle& nh)
    : StaticLayer(name), nh_(nh) {}

void LidarStaticLayer::clear() {
    for (auto& pub : lidar_publishers_) pub.shutdown();
    lidar_publishers_.clear();
}

void LidarStaticLayer::processLidarFile(const std::string& path, ros::Publisher& pub,
                                        double off_x, double off_y, double off_z) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    uint32_t cluster_num = 0;
    ifs.read(reinterpret_cast<char*>(&cluster_num), sizeof(uint32_t));

    for (uint32_t i = 0; i < cluster_num; ++i) {
        int32_t id;
        uint32_t point_num;
        ifs.read(reinterpret_cast<char*>(&id), sizeof(int32_t));
        ifs.read(reinterpret_cast<char*>(&point_num), sizeof(uint32_t));

        for (uint32_t j = 0; j < point_num; ++j) {
            float buffer[4];
            ifs.read(reinterpret_cast<char*>(buffer), sizeof(float) * 4);

            pcl::PointXYZI pt;
            pt.x = buffer[0] - off_x;
            pt.y = buffer[1] - off_y;
            pt.z = buffer[2] - off_z;

            float theta = buffer[3];
            while (theta < 0) theta += 2 * M_PI;
            while (theta >= M_PI) theta -= M_PI;
            pt.intensity = theta;
            cloud->push_back(pt);
        }
    }

    if (cloud->empty()) return;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.frame_id = "map";
    output_msg.header.stamp = ros::Time::now();
    pub.publish(output_msg);
}

void LidarStaticLayer::loadData(const std::string& base_dir, double off_x, double off_y, double off_z) {
    int seq_idx = 0;
    while (true) {
        std::string filename = "lidar_seq_" + std::to_string(seq_idx) + ".bin";
        std::string full_path = base_dir + filename;
        if (!fileExists(full_path)) break;

        std::string topic_name = "/lidar_viz/seq_" + std::to_string(seq_idx);
        ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true);
        lidar_publishers_.push_back(pub);
        processLidarFile(full_path, pub, off_x, off_y, off_z);
        seq_idx++;
    }
}
