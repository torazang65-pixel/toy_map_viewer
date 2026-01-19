#include <viewer/viewer_animation_loader.h>

#include <common/io.h>
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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

    std::string pkg_path = ros::package::getPath("realtime_line_generator");
    normalizeFolder(output_folder_);
    output_root_ = pkg_path + "/" + output_folder_ + std::to_string(file_idx_) + "/";

    frames_dir_ = output_root_ + "frames/";
    pred_frames_dir_ = output_root_ + "pred_frames/";
    voxels_dir_ = output_root_ + "voxels/";
    polylines_dir_ = output_root_ + "polylines/";
    merged_polylines_dir_ = output_root_ + "merged_polylines/";

    frame_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frame_points", 1);
    voxel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("active_voxels", 1);
    polyline_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_polylines", 1);
    merged_polyline_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("merged_polylines", 1);

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
}

void AnimationLoader::Run() {
    ros::Rate loop_rate(playback_rate_);
    int current_frame = start_frame_;

    while (ros::ok()) {
        while (ros::ok() && current_frame <= end_frame_) {
            publishFrame(current_frame);
            current_frame += frame_step_;
            ros::spinOnce();
            loop_rate.sleep();
        }
        if (!loop_playback_) {
            break;
        }
        current_frame = start_frame_;
    }
}

void AnimationLoader::normalizeFolder(std::string& folder) {
    if (!folder.empty() && folder.back() != '/') {
        folder.push_back('/');
    }
}

std::string AnimationLoader::framePath(const std::string& dir, int index) const {
    return dir + "frame_" + std::to_string(index) + ".bin";
}

int AnimationLoader::maxFrameIndexInDir(const std::string& dir) const {
    if (!fs::exists(dir)) {
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
        } catch (...) {
        }
    }
    return max_index;
}

bool AnimationLoader::loadPointsIfExists(
    const std::string& path,
    std::vector<linemapdraft_builder::data_types::Point>& points) {
    if (!fs::exists(path)) {
        return false;
    }
    return linemapdraft_builder::io::load_points(path, points);
}

bool AnimationLoader::loadPolylinesIfExists(
    const std::string& path,
    std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines) {
    if (!fs::exists(path)) {
        return false;
    }
    return linemapdraft_builder::io::load_polylines(path, polylines);
}

void AnimationLoader::publishPointCloud(
    const std::vector<linemapdraft_builder::data_types::Point>& points,
    ros::Publisher& pub) {
    if (points.empty()) {
        return;
    }

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_i(msg, "intensity");

    for (const auto& p : points) {
        *out_x = p.x - offset_.x;
        *out_y = p.y - offset_.y;
        *out_z = p.z;
        *out_i = p.yaw;
        ++out_x;
        ++out_y;
        ++out_z;
        ++out_i;
    }

    pub.publish(msg);
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
        bool loaded = loadPointsIfExists(frame_path, frame_points);
        if (!loaded && !use_pred_frames_) {
            loaded = loadPointsIfExists(framePath(pred_frames_dir_, frame_index), frame_points);
        }
        if (loaded) {
            MaybeInitOffsetFromPoints(offset_, frame_points);
            publishPointCloud(frame_points, frame_pub_);
        }
    }
    if (publish_voxels_ && loadPointsIfExists(voxel_path, voxel_points)) {
        MaybeInitOffsetFromPoints(offset_, voxel_points);
        publishPointCloud(voxel_points, voxel_pub_);
    }
    if (publish_polylines_ && loadPolylinesIfExists(polyline_path, polylines)) {
        MaybeInitOffsetFromPolylines(offset_, polylines);
        publishPolylines(polylines, polyline_pub_, "polylines", 0.0f, 1.0f, 0.0f);
    }
    if (publish_merged_polylines_ && loadPolylinesIfExists(merged_path, merged_polylines)) {
        MaybeInitOffsetFromPolylines(offset_, merged_polylines);
        publishPolylines(merged_polylines, merged_polyline_pub_, "merged_polylines", 1.0f, 0.2f, 0.2f);
    }
}

}  // namespace realtime_line_generator::viewer
