#include <viewer/viewer_animation_loader.h>
#include <viewer/viewer_utils.h>

#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <filesystem>

namespace fs = std::filesystem;

namespace realtime_line_generator::viewer {

AnimationLoader::AnimationLoader(ros::NodeHandle& nh, OffsetState& offset)
    : nh_(nh), offset_(offset) {
    nh_.param<std::string>("output_folder", output_folder_, "data/issue/converted_bin/");
    nh_.param("file_idx", file_idx_, 20000);
    nh_.param("start_frame", start_frame_, 0);
    nh_.param("frame_step", frame_step_, 1);
    nh_.param("playback_rate", playback_rate_, 10.0);
    nh_.param("loop_playback", loop_playback_, true);
    nh_.param<std::string>("frame_id", frame_id_, "map");
    nh_.param("publish_frame_points", publish_frame_points_, true);
    nh_.param("use_pred_frames", use_pred_frames_, false);
    nh_.param("publish_voxels", publish_voxels_, true);
    nh_.param("publish_polylines", publish_polylines_, true);
    nh_.param("publish_merged_polylines", publish_merged_polylines_, true);
    nh_.param("publish_vehicle_pose", publish_vehicle_pose_, true);

    std::string pkg_path = ros::package::getPath("realtime_line_generator");
    NormalizeFolder(output_folder_);
    output_root_ = pkg_path + "/" + output_folder_ + std::to_string(file_idx_) + "/";

    frames_dir_ = output_root_ + "frames/";
    pred_frames_dir_ = output_root_ + "pred_frames/";
    voxels_dir_ = output_root_ + "voxels/";
    polylines_dir_ = output_root_ + "polylines/";
    merged_polylines_dir_ = output_root_ + "merged_polylines/";
    vehicle_trajectory_path_ = output_root_ + "vehicle_trajectory.bin";

    frame_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frame_points", 1);
    voxel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("active_voxels", 1);
    polyline_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_polylines", 1);
    merged_polyline_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("merged_polylines", 1);
    vehicle_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_pose", 1);

    if (frame_step_ < 1) {
        frame_step_ = 1;
    }

    const std::string& frame_dir = use_pred_frames_ ? pred_frames_dir_ : frames_dir_;
    end_frame_ = maxFrameIndexInDir(frame_dir);
    if (end_frame_ < start_frame_ && !use_pred_frames_) {
        end_frame_ = maxFrameIndexInDir(pred_frames_dir_);
    }
    if (end_frame_ < start_frame_) {
        ROS_WARN("No frame files found under %s", frame_dir.c_str());
    }

    current_frame_ = start_frame_;

    if (publish_vehicle_pose_) {
        if (!LoadPointsIfExists(vehicle_trajectory_path_, vehicle_trajectory_)) {
            ROS_WARN("Vehicle trajectory not found: %s", vehicle_trajectory_path_.c_str());
        }
    }
}

void AnimationLoader::Start() {
    current_frame_ = start_frame_;
    is_playing_ = true;
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / playback_rate_),
        &AnimationLoader::timerCallback, this);
    ROS_INFO("Animation started from frame %d", current_frame_);
}

void AnimationLoader::Stop() {
    timer_.stop();
    is_playing_ = false;
    current_frame_ = start_frame_;
    ROS_INFO("Animation stopped");
}

void AnimationLoader::Pause() {
    timer_.stop();
    is_playing_ = false;
    ROS_INFO("Animation paused at frame %d", current_frame_);
}

void AnimationLoader::Resume() {
    if (!is_playing_) {
        is_playing_ = true;
        timer_.start();
        ROS_INFO("Animation resumed from frame %d", current_frame_);
    }
}

void AnimationLoader::Seek(int frame_index) {
    if (frame_index >= start_frame_ && frame_index <= end_frame_) {
        current_frame_ = frame_index;
        publishFrame(current_frame_);
        ROS_INFO("Seeked to frame %d", current_frame_);
    } else {
        ROS_WARN("Seek frame %d out of range [%d, %d]", frame_index, start_frame_, end_frame_);
    }
}

void AnimationLoader::timerCallback(const ros::TimerEvent& event) {
    if (current_frame_ > end_frame_) {
        if (!loop_playback_) {
            Stop();
            return;
        }
        current_frame_ = start_frame_;
    }
    publishFrame(current_frame_);
    current_frame_ += frame_step_;
}

std::string AnimationLoader::framePath(const std::string& dir, int index) const {
    return dir + "frame_" + std::to_string(index) + ".bin";
}

int AnimationLoader::maxFrameIndexInDir(const std::string& dir) const {
    if (!fs::exists(dir)) {
        ROS_DEBUG("Directory does not exist: %s", dir.c_str());
        return -1;
    }
    int max_index = -1;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto& path = entry.path();
        if (path.extension() != ".bin") {
            continue;
        }
        const std::string stem = path.stem().string();
        const std::string prefix = "frame_";
        if (stem.rfind(prefix, 0) != 0) {
            continue;
        }
        const std::string index_str = stem.substr(prefix.size());
        try {
            int idx = std::stoi(index_str);
            if (idx > max_index) {
                max_index = idx;
            }
        } catch (const std::exception& e) {
            ROS_WARN("Failed to parse frame index from '%s': %s", stem.c_str(), e.what());
        }
    }
    return max_index;
}

void AnimationLoader::publishPolylines(
    const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines,
    ros::Publisher& pub,
    const std::string& ns,
    float r, float g, float b) {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < polylines.size(); ++i) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = frame_id_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = ns;
        line_strip.id = static_cast<int>(i);
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.2f;
        line_strip.color.r = r;
        line_strip.color.g = g;
        line_strip.color.b = b;
        line_strip.color.a = 1.0f;

        for (const auto& pt : polylines[i]) {
            geometry_msgs::Point p;
            p.x = pt.x - offset_.x;
            p.y = pt.y - offset_.y;
            p.z = pt.z;
            line_strip.points.push_back(p);
        }
        marker_array.markers.push_back(line_strip);
    }

    pub.publish(marker_array);
}

void AnimationLoader::publishVehiclePose(int frame_index) {
    if (!publish_vehicle_pose_) {
        return;
    }
    if (vehicle_trajectory_.empty()) {
        return;
    }
    if (frame_index < 0 || static_cast<size_t>(frame_index) >= vehicle_trajectory_.size()) {
        return;
    }

    const auto& pose = vehicle_trajectory_[frame_index];
    MaybeInitOffsetFromPoints(offset_, vehicle_trajectory_);

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "vehicle_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.x - offset_.x;
    marker.pose.position.y = pose.y - offset_.y;
    marker.pose.position.z = pose.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.8f;
    marker.scale.y = 0.8f;
    marker.scale.z = 0.8f;
    marker.color.r = 0.2f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    vehicle_pose_pub_.publish(marker);
}

void AnimationLoader::publishFrame(int frame_index) {
    std::vector<linemapdraft_builder::data_types::Point> frame_points;
    std::vector<linemapdraft_builder::data_types::Point> voxel_points;
    std::vector<std::vector<linemapdraft_builder::data_types::Point>> polylines;
    std::vector<std::vector<linemapdraft_builder::data_types::Point>> merged_polylines;

    const std::string frame_path =
        framePath(use_pred_frames_ ? pred_frames_dir_ : frames_dir_, frame_index);
    const std::string voxel_path = framePath(voxels_dir_, frame_index);
    const std::string polyline_path = framePath(polylines_dir_, frame_index);
    const std::string merged_path = framePath(merged_polylines_dir_, frame_index);

    if (publish_frame_points_) {
        bool loaded = LoadPointsIfExists(frame_path, frame_points);
        if (!loaded && !use_pred_frames_) {
            loaded = LoadPointsIfExists(framePath(pred_frames_dir_, frame_index), frame_points);
        }
        if (loaded) {
            MaybeInitOffsetFromPoints(offset_, frame_points);
            PublishPointCloud(frame_points, offset_, frame_id_, frame_pub_);
        }
    }
    if (publish_voxels_ && LoadPointsIfExists(voxel_path, voxel_points)) {
        MaybeInitOffsetFromPoints(offset_, voxel_points);
        PublishPointCloud(voxel_points, offset_, frame_id_, voxel_pub_);
    }
    if (publish_polylines_ && LoadPolylinesIfExists(polyline_path, polylines)) {
        MaybeInitOffsetFromPolylines(offset_, polylines);
        publishPolylines(polylines, polyline_pub_, "polylines", 0.0f, 1.0f, 0.0f);
    }
    if (publish_merged_polylines_ && LoadPolylinesIfExists(merged_path, merged_polylines)) {
        MaybeInitOffsetFromPolylines(offset_, merged_polylines);
        publishPolylines(merged_polylines, merged_polyline_pub_, "merged_polylines", 1.0f, 0.2f, 0.2f);
    }
    publishVehiclePose(frame_index);
}

}  // namespace realtime_line_generator::viewer
