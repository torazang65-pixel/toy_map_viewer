#include <viewer/viewer_animation_loader.h>
#include <viewer/viewer_utils.h>

#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <unordered_map>

namespace fs = std::filesystem;

namespace realtime_line_generator::viewer {

namespace {
uint32_t HashId(uint32_t id) {
    uint32_t x = id;
    x = x * 2654435761u;
    x = ((x >> 16) ^ x) * 0x45d9f3bu;
    x = ((x >> 16) ^ x) * 0x45d9f3bu;
    x = (x >> 16) ^ x;
    return x;
}

std::array<float, 3> HsvToRgb(float h, float s, float v) {
    if (s == 0.0f) {
        return {v, v, v};
    }
    int i = static_cast<int>(h * 6.0f);
    float f = (h * 6.0f) - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    i %= 6;
    switch (i) {
        case 0: return {v, t, p};
        case 1: return {q, v, p};
        case 2: return {p, v, t};
        case 3: return {p, q, v};
        case 4: return {t, p, v};
        default: return {v, p, q};
    }
}

std::array<float, 3> ColorForId(int id, int seq_idx) {
    uint32_t seed = static_cast<uint32_t>(id) ^ (static_cast<uint32_t>(seq_idx) * 0x9e3779b9u);
    uint32_t hashed_id = HashId(seed);
    float hue = std::fmod(static_cast<float>(hashed_id) * 0.61803398875f, 1.0f);
    return HsvToRgb(hue, 0.85f, 0.95f);
}
}  // namespace

namespace ldb = linemapdraft_builder;

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
    nh_.param("publish_gt_frame_prev", publish_gt_frame_prev_, true);
    nh_.param("publish_gt_frame_latest", publish_gt_frame_latest_, true);
    nh_.param("publish_vehicle_pose", publish_vehicle_pose_, true);

    std::string pkg_path = ros::package::getPath("realtime_line_generator");
    NormalizeFolder(output_folder_);
    output_root_ = pkg_path + "/" + output_folder_ + std::to_string(file_idx_) + "/";

    frames_dir_ = output_root_ + "frames/";
    pred_frames_dir_ = output_root_ + "pred_frames/";
    voxels_dir_ = output_root_ + "voxels/";
    polylines_dir_ = output_root_ + "polylines/";
    merged_polylines_dir_ = output_root_ + "merged_polylines/";
    gt_frame_prev_dir_ = output_root_ + "gt_frame_prev/";
    gt_frame_latest_dir_ = output_root_ + "gt_frame_latest/";
    vehicle_trajectory_path_ = output_root_ + "vehicle_trajectory.bin";

    frame_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frame_points", 1);
    voxel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("active_voxels", 1);
    polyline_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_polylines", 1);
    merged_polyline_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("merged_polylines", 1);
    gt_prev_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gt_frame_prev", 1);
    gt_latest_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gt_frame_latest", 1);
    vehicle_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_pose", 1);

    // 평가 결과 발행 여부 파라미터 (선택 사항)
    nh_.param("publish_eval", publish_eval_, true);

    // 빌더에서 저장한 경로와 일치하도록 설정
    eval_fp_dir_ = output_root_ + "eval_fp/fp_";
    eval_fn_dir_ = output_root_ + "eval_fn/fn_";

    // 마커 퍼블리셔 선언
    eval_fp_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("eval/false_positives", 1);
    eval_fn_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("eval/false_negatives", 1);

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

void AnimationLoader::publishPointsByPolylineId(
    const std::vector<linemapdraft_builder::data_types::Point>& points,
    ros::Publisher& pub,
    const std::string& ns,
    float scale,
    int seq_idx) {
    if (points.empty()) {
        return;
    }

    std::unordered_map<int32_t, std::vector<geometry_msgs::Point>> grouped;
    grouped.reserve(points.size());

    for (const auto& p : points) {
        geometry_msgs::Point pt;
        pt.x = p.x - offset_.x;
        pt.y = p.y - offset_.y;
        pt.z = p.z;
        grouped[p.polyline_id].push_back(pt);
    }

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int marker_id = 0;
    for (const auto& [id, pts] : grouped) {
        if (pts.empty()) continue;

        auto [r, g, b] = ColorForId(id, seq_idx);
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;
        marker.points = pts;
        marker_array.markers.push_back(marker);
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
    std::vector<linemapdraft_builder::data_types::Point> gt_prev_points;
    std::vector<linemapdraft_builder::data_types::Point> gt_latest_points;
    std::vector<std::vector<linemapdraft_builder::data_types::Point>> polylines;
    std::vector<std::vector<linemapdraft_builder::data_types::Point>> merged_polylines;
    std::vector<std::vector<ldb::data_types::Point>> fp_lines, fn_lines;

    const std::string frame_path =
        framePath(use_pred_frames_ ? pred_frames_dir_ : frames_dir_, frame_index);
    const std::string voxel_path = framePath(voxels_dir_, frame_index);
    const std::string polyline_path = framePath(polylines_dir_, frame_index);
    const std::string merged_path = framePath(merged_polylines_dir_, frame_index);
    const std::string gt_prev_path = framePath(gt_frame_prev_dir_, frame_index);
    const std::string gt_latest_path = framePath(gt_frame_latest_dir_, frame_index);
    const std::string fp_path = eval_fp_dir_ + std::to_string(frame_index) + ".bin";
    const std::string fn_path = eval_fn_dir_ + std::to_string(frame_index) + ".bin";

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
    if (publish_gt_frame_prev_ && LoadPointsIfExists(gt_prev_path, gt_prev_points)) {
        MaybeInitOffsetFromPoints(offset_, gt_prev_points);
        publishPointsByPolylineId(gt_prev_points, gt_prev_pub_, "gt_frame_prev", 0.2f, 0);
    }
    if (publish_gt_frame_latest_ && LoadPointsIfExists(gt_latest_path, gt_latest_points)) {
        MaybeInitOffsetFromPoints(offset_, gt_latest_points);
        publishPointsByPolylineId(gt_latest_points, gt_latest_pub_, "gt_frame_latest", 0.2f, 1);
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
    

    if (publish_eval_) {
    std::vector<std::vector<ldb::data_types::Point>> fp_lines, fn_lines;
    
    // FP(오탐) 로드 및 빨간색 시각화
    if (loadPolylinesIfExists(fp_path, fp_lines)) {
        publishPolylines(fp_lines, eval_fp_pub_, "false_positives", 1.0f, 0.0f, 0.0f);
    }
    
    // FN(미탐) 로드 및 파란색 시각화
    if (loadPolylinesIfExists(fn_path, fn_lines)) {
        publishPolylines(fn_lines, eval_fn_pub_, "false_negatives", 0.0f, 0.0f, 1.0f);
    }
}
}

}  // namespace realtime_line_generator::viewer
