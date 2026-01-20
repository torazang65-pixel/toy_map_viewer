#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

#include <viewer/viewer_offset.h>

namespace realtime_line_generator::viewer {

class AnimationLoader {
public:
    AnimationLoader(ros::NodeHandle& nh, OffsetState& offset);

    void Run();

private:
    void normalizeFolder(std::string& folder);
    std::string framePath(const std::string& dir, int index) const;
    int maxFrameIndexInDir(const std::string& dir) const;
    bool loadPointsIfExists(
        const std::string& path,
        std::vector<linemapdraft_builder::data_types::Point>& points);
    bool loadPolylinesIfExists(
        const std::string& path,
        std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines);
    void publishPointCloud(const std::vector<linemapdraft_builder::data_types::Point>& points,
                           ros::Publisher& pub);
    void publishPolylines(const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines,
                          ros::Publisher& pub,
                          const std::string& ns,
                          float r, float g, float b);
    void publishVehiclePose(int frame_index);
    void publishFrame(int frame_index);

    ros::NodeHandle nh_;
    OffsetState& offset_;

    ros::Publisher frame_pub_;
    ros::Publisher voxel_pub_;
    ros::Publisher polyline_pub_;
    ros::Publisher merged_polyline_pub_;
    ros::Publisher vehicle_pose_pub_;

    std::string output_folder_;
    std::string output_root_;
    std::string frames_dir_;
    std::string pred_frames_dir_;
    std::string voxels_dir_;
    std::string polylines_dir_;
    std::string merged_polylines_dir_;
    std::string vehicle_trajectory_path_;
    std::string frame_id_;
    std::vector<linemapdraft_builder::data_types::Point> vehicle_trajectory_;
    int file_idx_ = 0;
    int start_frame_ = 0;
    int end_frame_ = 0;
    int frame_step_ = 1;
    double playback_rate_ = 10.0;
    bool loop_playback_ = true;
    bool publish_frame_points_ = true;
    bool use_pred_frames_ = false;
    bool publish_voxels_ = true;
    bool publish_polylines_ = true;
    bool publish_merged_polylines_ = true;
    bool publish_vehicle_pose_ = true;

    bool publish_eval_;
    std::string eval_fp_dir_;
    std::string eval_fn_dir_;
    ros::Publisher eval_fp_pub_;
    ros::Publisher eval_fn_pub_;
};

}  // namespace realtime_line_generator::viewer
