#pragma once

#include "AnimationLayer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudLayer : public AnimationLayer {
public:
    PointCloudLayer(const std::string& name, ros::NodeHandle& nh, const std::string& topic, 
                    const std::string& sub_folder, const std::string& file_prefix, 
                    bool has_density_field, bool visualize_density);
    
    ~PointCloudLayer() override = default;

    void loadData(const std::string& base_dir, double off_x, double off_y, double off_z) override;
    void publish(size_t frame_idx, const std_msgs::Header& header) override;
    void clear() override;

protected:
    void publishEmpty() override;

private:
    ros::Publisher pub_;
    std::string sub_folder_;
    std::string file_prefix_;

    bool has_density_field_;
    bool visualize_density_;
    
    // 데이터 저장소
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> frames_;
};