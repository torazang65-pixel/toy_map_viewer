#ifndef REALTIME_LINE_GENERATOR_REALTIME_LINE_BUILDER_H
#define REALTIME_LINE_GENERATOR_REALTIME_LINE_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <realtime_line_generator/realtime_voxel_manager.h>
#include <common/data_types.h>
#include <cstddef>

namespace realtime_line_generator {

class RealTimeLineBuilder {
public:
    RealTimeLineBuilder(ros::NodeHandle& nh);
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    void processPipeline(const std::vector<linemapdraft_builder::data_types::Point>& frame_points, const std::string& frame_id);
    void processPipelineFrame(const std::vector<linemapdraft_builder::data_types::Point>& frame_points, const std::string& frame_id);

    void processGeneratorStage(std::vector<linemapdraft_builder::data_types::Point>& active_voxels, const std::string& frame_id);
    void logTimingSummary() const;

    const std::vector<linemapdraft_builder::data_types::Point>& getLastFramePoints() const;
    const std::vector<linemapdraft_builder::data_types::Point>& getLastActiveVoxels() const;
    const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& getLastPolylines() const;
    const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& getLastMergedPolylines() const;
    
private:
    
    std::unique_ptr<RealTimeVoxelManager> voxel_manager_;
    uint32_t frame_counter_ = 0;
    float voxel_size_;
    int yaw_voxel_num_;
    int min_density_th_; // nh.param 호환을 위해 int로 변경 권장
    int max_voxel_age_;  // int로 변경 권장
    int voxel_aging_interval_;
    bool use_voxel_aging_;

    // polyline_generator
    float neighbor_dist_thresh_;
    float cylinder_search_width_;
    float alpha_, beta_, drop_width_;

    // polyline_simplifier
    bool use_vw_;
    bool use_rdp_;
    float vw_eps_;
    float vw_eps_2_;
    float rdp_eps_;

    // post processing
    float merge_min_dist_th_, merge_max_dist_th_;
    float merge_min_angle_th_, merge_max_angle_th_;
    float min_polyline_length_;

    std::vector<linemapdraft_builder::data_types::Point> last_frame_points_;
    std::vector<linemapdraft_builder::data_types::Point> last_active_voxels_;
    std::vector<std::vector<linemapdraft_builder::data_types::Point>> last_polylines_;
    std::vector<std::vector<linemapdraft_builder::data_types::Point>> last_merged_polylines_;

    std::size_t timing_frame_count_ = 0;
    double timing_polyline_generator_ms_ = 0.0;
    double timing_polyline_simplifier_ms_ = 0.0;
    double timing_polyline_merge_ms_ = 0.0;

};

} // namespace realtime_line_generator
#endif
