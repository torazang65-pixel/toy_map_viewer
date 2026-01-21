#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

#include <common/data_types.h>
#include <viewer/viewer_offset.h>

namespace realtime_line_generator::viewer {

class AnimationLoader {
public:
    AnimationLoader(ros::NodeHandle& nh, OffsetState& offset);

    void Start();
    void Stop();
    void Pause();
    void Resume();
    bool IsPlaying() const { return is_playing_; }
    int CurrentFrame() const { return current_frame_; }
    void Seek(int frame_index);

private:
    void timerCallback(const ros::TimerEvent& event);
    std::string framePath(const std::string& dir, int index) const;
    int maxFrameIndexInDir(const std::string& dir) const;
    void publishPolylines(const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines,
                          ros::Publisher& pub,
                          const std::string& ns,
                          float r, float g, float b);
    void publishPointsByPolylineId(const std::vector<linemapdraft_builder::data_types::Point>& points,
                                   ros::Publisher& pub,
                                   const std::string& ns,
                                   float scale,
                                   int seq_idx);
    void publishVehiclePose(int frame_index);
    void publishFrame(int frame_index);

    ros::NodeHandle nh_;
    OffsetState& offset_;

    ros::Timer timer_;
    ros::Publisher frame_pub_;
    ros::Publisher voxel_pub_;
    ros::Publisher polyline_pub_;
    ros::Publisher merged_polyline_pub_;
    ros::Publisher gt_prev_pub_;
    ros::Publisher gt_latest_pub_;
    ros::Publisher vehicle_pose_pub_;

    std::string output_folder_;
    std::string output_root_;
    std::string frames_dir_;
    std::string pred_frames_dir_;
    std::string voxels_dir_;
    std::string polylines_dir_;
    std::string merged_polylines_dir_;
    std::string gt_frame_prev_dir_;
    std::string gt_frame_latest_dir_;
    std::string vehicle_trajectory_path_;
    std::string frame_id_;
    std::vector<linemapdraft_builder::data_types::Point> vehicle_trajectory_;
    int file_idx_ = 0;
    int start_frame_ = 0;
    int end_frame_ = 0;
    int current_frame_ = 0;
    int frame_step_ = 1;
    double playback_rate_ = 10.0;
    bool loop_playback_ = true;
    bool is_playing_ = false;
    bool publish_frame_points_ = true;
    bool use_pred_frames_ = false;
    bool publish_voxels_ = true;
    bool publish_polylines_ = true;
    bool publish_merged_polylines_ = true;
    bool publish_gt_frame_prev_ = true;
    bool publish_gt_frame_latest_ = true;
    bool publish_vehicle_pose_ = true;
};

}  // namespace realtime_line_generator::viewer
