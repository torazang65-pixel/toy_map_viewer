#include <viewer/viewer_static_map_loader.h>
#include <viewer/viewer_utils.h>

#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <regex>
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

namespace fs = std::filesystem;

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

    // map_points_pub_ is fixed
    map_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("mapconverter_points", 1, true);

    // Dynamic publishers will be created on demand in Publish() to avoid scanning twice
}

void StaticMapLoader::Publish() {
    if (!fs::exists(converted_root_)) {
        ROS_WARN("Output directory does not exist: %s", converted_root_.c_str());
        return;
    }

    std::vector<linemapdraft_builder::data_types::Point> all_map_points;
    bool found_map_points = false;

    // Regex for matching files
    std::regex points_seq_regex(R"(points_seq_(\d+)\.bin)");
    std::regex lidar_seq_regex(R"(lidar_seq_(\d+)\.bin)");
    std::smatch match;

    for (const auto& entry : fs::directory_iterator(converted_root_)) {
        if (!entry.is_regular_file()) continue;

        std::string filename = entry.path().filename().string();
        std::string full_path = entry.path().string();

        // 1. Process points_seq_X.bin
        if (std::regex_match(filename, match, points_seq_regex)) {
            if (publish_map_points_) {
                std::vector<linemapdraft_builder::data_types::Point> pts;
                if (LoadPointsIfExists(full_path, pts)) {
                    all_map_points.insert(all_map_points.end(), pts.begin(), pts.end());
                    found_map_points = true;
                }
            }
        }
        // 2. Process lidar_seq_X.bin
        else if (std::regex_match(filename, match, lidar_seq_regex)) {
            int seq_idx = std::stoi(match[1].str());
            std::string topic_name;

            // Determine topic name based on sequence index (Legacy support)
            if (seq_idx == 0) {
                if (!publish_converted_filtered_) continue;
                topic_name = "converted_map_filtered";
            } else if (seq_idx == 1) {
                if (!publish_converted_raw_) continue;
                topic_name = "converted_map_raw";
            } else {
                topic_name = "converted_map_seq_" + std::to_string(seq_idx);
            }

            // Create publisher if not exists
            if (lidar_pubs_.find(topic_name) == lidar_pubs_.end()) {
                lidar_pubs_[topic_name] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true);
            }

            std::vector<linemapdraft_builder::data_types::Point> converted_points;
            if (LoadPointsIfExists(full_path, converted_points)) {
                MaybeInitOffsetFromPoints(offset_, converted_points);
                PublishPointCloud(converted_points, offset_, frame_id_, lidar_pubs_[topic_name]);
            }
        }
    }

    // Publish amassed map points
    if (publish_map_points_ && found_map_points) {
        MaybeInitOffsetFromPoints(offset_, all_map_points);
        publishPointsByPolylineId(all_map_points, map_points_pub_, "mapconverter_points", 0.2f);
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
