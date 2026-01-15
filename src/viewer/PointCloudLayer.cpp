#include "viewer/PointCloudLayer.h"
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <sys/stat.h>

// 내부 유틸리티
namespace {
    bool fileExists(const std::string& name) {
        struct stat buffer; return (stat (name.c_str(), &buffer) == 0); 
    }
}

PointCloudLayer::PointCloudLayer(const std::string& name, ros::NodeHandle& nh, const std::string& topic, 
                                 const std::string& sub_folder, const std::string& file_prefix, bool has_density_field, bool visualize_density)
    : AnimationLayer(name), sub_folder_(sub_folder), file_prefix_(file_prefix), has_density_field_(has_density_field), visualize_density_(visualize_density) {
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
}

void PointCloudLayer::loadData(const std::string& base_dir, double off_x, double off_y, double off_z) {
    clear();
    int idx = 0;
    
    ROS_INFO("[%s] Loading data from %s...", name_.c_str(), sub_folder_.c_str());

    while(true) {
        std::string path = base_dir + sub_folder_ + "/" + file_prefix_ + "_" + std::to_string(idx) + ".bin";
        if (!fileExists(path)) break;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        std::ifstream ifs(path, std::ios::binary);
        
        if (ifs.is_open()) {
            uint32_t cluster_num = 0;
            ifs.read(reinterpret_cast<char*>(&cluster_num), 4);
            
            for (uint32_t c = 0; c < cluster_num; ++c) {
                int32_t id; uint32_t p_num;
                ifs.read(reinterpret_cast<char*>(&id), 4);
                ifs.read(reinterpret_cast<char*>(&p_num), 4);
                
                for (uint32_t j = 0; j < p_num; ++j) {
                    float x, y, z, val; // val holds intensity or yaw(Voxel)
                    uint32_t density = 0;
                    
                    ifs.read(reinterpret_cast<char*>(&x), 4);
                    ifs.read(reinterpret_cast<char*>(&y), 4);
                    ifs.read(reinterpret_cast<char*>(&z), 4);
                    ifs.read(reinterpret_cast<char*>(&val), 4);
                    if (has_density_field_) ifs.read(reinterpret_cast<char*>(&density), 4);

                    pcl::PointXYZI pt;
                    pt.x = x - off_x; 
                    pt.y = y - off_y; 
                    pt.z = z - off_z;
                    if (visualize_density_) pt.intensity = static_cast<float>(density);
                    else pt.intensity = val;

                    cloud->push_back(pt);
                }
            }
            ifs.close();
        }
        frames_.push_back(cloud);
        idx++;
    }
    frame_count_ = frames_.size();
    ROS_INFO("[%s] Loaded %lu frames.", name_.c_str(), frame_count_);
}

void PointCloudLayer::publish(size_t frame_idx, const std_msgs::Header& header) {
    if (!visible_ || frames_.empty()) return;
    
    size_t idx = frame_idx % frames_.size();
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*frames_[idx], msg);
    msg.header = header;
    pub_.publish(msg);
}

void PointCloudLayer::publishEmpty() {
    sensor_msgs::PointCloud2 empty;
    empty.header.frame_id = "map";
    empty.header.stamp = ros::Time::now();
    pub_.publish(empty);
}

void PointCloudLayer::clear() {
    frames_.clear();
    frame_count_ = 0;
}