#include <viewer/viewer_static_map_loader.h>
#include <viewer/viewer_utils.h>

#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>

#include <array>
#include <cmath>
#include <unordered_map>

namespace realtime_line_generator::viewer {

namespace {
uint32_t HashId(int id) {
    uint32_t x = static_cast<uint32_t>(id);
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

std::array<float, 3> ColorForId(int id) {
    uint32_t hashed_id = HashId(id);
    float hue = std::fmod(static_cast<float>(hashed_id) * 0.61803398875f, 1.0f);
    return HsvToRgb(hue, 0.85f, 0.95f);
}
}  // namespace

StaticMapLoader::StaticMapLoader(ros::NodeHandle& nh, OffsetState& offset)
    : nh_(nh), offset_(offset) {
    nh_.param<std::string>("output_folder", output_folder_, "data/issue/converted_bin/");
    nh_.param("file_idx", file_idx_, 20000);
    nh_.param<std::string>("frame_id", frame_id_, "map");
    nh_.param("publish_map_points", publish_map_points_, true);
    nh_.param("publish_converted_filtered", publish_converted_filtered_, true);
    nh_.param("publish_converted_raw", publish_converted_raw_, true);

    std::string pkg_path = ros::package::getPath("realtime_line_generator");
    NormalizeFolder(output_folder_);
    output_root_ = pkg_path + "/" + output_folder_ + std::to_string(file_idx_) + "/";
    converted_root_ = output_root_;
    map_points_path_ = output_root_ + "points_seq_0.bin";

    map_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("mapconverter_points", 1, true);
    converted_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("converted_map_filtered", 1, true);
    converted_map_raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("converted_map_raw", 1, true);
}

void StaticMapLoader::Publish() {
    std::vector<linemapdraft_builder::data_types::Point> map_points;
    if (publish_map_points_ && LoadPointsIfExists(map_points_path_, map_points)) {
        MaybeInitOffsetFromPoints(offset_, map_points);
        publishPointsByPolylineId(map_points, map_points_pub_, "mapconverter_points", 0.2f);
    }

    std::vector<linemapdraft_builder::data_types::Point> converted_points;
    std::string converted_filtered_path = converted_root_ + "lidar_seq_0.bin";
    if (publish_converted_filtered_ && LoadPointsIfExists(converted_filtered_path, converted_points)) {
        MaybeInitOffsetFromPoints(offset_, converted_points);
        PublishPointCloud(converted_points, offset_, frame_id_, converted_map_pub_);
    }

    converted_points.clear();
    std::string converted_raw_path = converted_root_ + "lidar_seq_1.bin";
    if (publish_converted_raw_ && LoadPointsIfExists(converted_raw_path, converted_points)) {
        MaybeInitOffsetFromPoints(offset_, converted_points);
        PublishPointCloud(converted_points, offset_, frame_id_, converted_map_raw_pub_);
    }
}

void StaticMapLoader::publishPointsByPolylineId(
    const std::vector<linemapdraft_builder::data_types::Point>& points,
    ros::Publisher& pub,
    const std::string& ns,
    float scale) {
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

        auto [r, g, b] = ColorForId(id);
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

}  // namespace realtime_line_generator::viewer
