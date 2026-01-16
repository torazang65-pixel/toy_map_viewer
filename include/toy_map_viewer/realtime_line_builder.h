#ifndef TOY_MAP_VIEWER_REALTIME_LINE_BUILDER_H
#define TOY_MAP_VIEWER_REALTIME_LINE_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <toy_map_viewer/realtime_voxel_manager.h>
#include <common/data_types.h>
#include <visualization_msgs/MarkerArray.h>

namespace toy_map_viewer {

class RealTimeLineBuilder {
public:
    RealTimeLineBuilder(ros::NodeHandle& nh);
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    void processPipeline(const std::vector<linemapdraft_builder::data_types::Point>& frame_points, const std::string& frame_id);

    void processGeneratorStage(std::vector<linemapdraft_builder::data_types::Point>& active_voxels, const std::string& frame_id);
    
private:
    
    // voxel
    void publishVoxelCloud(const std::vector<linemapdraft_builder::data_types::Point>& voxels, const std::string& frame_id);

    std::unique_ptr<RealTimeVoxelManager> voxel_manager_;
    ros::Publisher voxel_pub_;
    uint32_t frame_counter_ = 0;
    float voxel_size_;
    int yaw_voxel_num_;
    int min_density_th_; // nh.param 호환을 위해 int로 변경 권장
    int max_voxel_age_;  // int로 변경 권장

    // polyline_generator
    void publishPolylines(const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines, const std::string& frame_id);

    ros::Publisher polyline_pub_;
    
    float neighbor_dist_thresh_;
    float cylinder_search_width_;
    float alpha_, beta_, drop_width_;

    // polyline_simplifier
    bool use_vw_;
    bool use_rdp_;
    float vw_eps_;
    float rdp_eps_;

    // post processing
    float merge_min_dist_th_, merge_max_dist_th_;
    float merge_min_angle_th_, merge_max_angle_th_;
    float min_polyline_length_;

};

} // namespace toy_map_viewer
#endif