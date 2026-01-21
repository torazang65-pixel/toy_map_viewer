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

    map_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("mapconverter_points", 1, true);
    converted_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("converted_map_filtered", 1, true);
    converted_map_raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("converted_map_raw", 1, true);

    gt_map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gt_map", 1, true);
    gt_path_ = output_root_ + "gt.bin";
    // Dynamic publishers will be created on demand in Publish()
}

void StaticMapLoader::Publish() {
    std::vector<linemapdraft_builder::data_types::Point> map_points;
    if (publish_map_points_ && loadPointsIfExists(map_points_path_, map_points)) {
        MaybeInitOffsetFromPoints(offset_, map_points);
        publishPointsByPolylineId(map_points, map_points_pub_, "mapconverter_points", 0.2f);
    }

    // gt.bin 시각화 로직
    std::vector<std::vector<linemapdraft_builder::data_types::Point>> gt_polylines;
    if (LoadPolylinesIfExists(gt_path_, gt_polylines)) {
        MaybeInitOffsetFromPolylines(offset_, gt_polylines);
        // 노란색(1.0, 1.0, 0.0) 선으로 발행
        publishPolylines(gt_polylines, gt_map_pub_, "gt_map", 1.0f, 1.0f, 0.0f);
    }
    
    std::vector<linemapdraft_builder::data_types::Point> converted_points;
    std::string converted_filtered_path = converted_root_ + "lidar_seq_0.bin";
    if (publish_converted_filtered_ && loadPointsIfExists(converted_filtered_path, converted_points)) {
        MaybeInitOffsetFromPoints(offset_, converted_points);
        publishPointCloud(converted_points, converted_map_pub_);
    }

    converted_points.clear();
    std::string converted_raw_path = converted_root_ + "lidar_seq_1.bin";
    if (publish_converted_raw_ && loadPointsIfExists(converted_raw_path, converted_points)) {
        MaybeInitOffsetFromPoints(offset_, converted_points);
        publishPointCloud(converted_points, converted_map_raw_pub_);
    }
}

void StaticMapLoader::normalizeFolder(std::string& folder) {
    if (!folder.empty() && folder.back() != '/') {
        folder.push_back('/');
    }
}

bool StaticMapLoader::loadPointsIfExists(
    const std::string& path,
    std::vector<linemapdraft_builder::data_types::Point>& points) {
    if (!fs::exists(path)) {
        return false;
    }
    return linemapdraft_builder::io::load_points(path, points);
}

bool StaticMapLoader::loadPolylinesIfExists(
    const std::string& path,
    std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines) {
    if (!fs::exists(path)) return false;
    return linemapdraft_builder::io::load_polylines(path, polylines);
}

// 선(Line Strip) 형태로 MarkerArray 발행 함수
void StaticMapLoader::publishPolylines(
    const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines,
    ros::Publisher& pub, const std::string& ns, float r, float g, float b) {
    
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < polylines.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.2; // 선 두께
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0f;

        for (const auto& p : polylines[i]) {
            geometry_msgs::Point pt;
            pt.x = p.x - offset_.x;
            pt.y = p.y - offset_.y;
            pt.z = p.z;
            marker.points.push_back(pt);
        }
        marker_array.markers.push_back(marker);
    }
    pub.publish(marker_array);
}

void StaticMapLoader::publishPointCloud(const std::vector<linemapdraft_builder::data_types::Point>& points,
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
    if (!fs::exists(converted_root_)) {
        ROS_WARN("Output directory does not exist: %s", converted_root_.c_str());
        return;
    }

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
            if (!publish_map_points_) {
                continue;
            }
            int seq_idx = std::stoi(match[1].str());
            std::string topic_name = "points_seq_" + std::to_string(seq_idx);

            if (points_pubs_.find(topic_name) == points_pubs_.end()) {
                points_pubs_[topic_name] =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
            }

            std::vector<linemapdraft_builder::data_types::Point> pts;
            if (LoadPointsIfExists(full_path, pts)) {
                MaybeInitOffsetFromPoints(offset_, pts);
                publishPointsByPolylineId(pts, points_pubs_[topic_name], topic_name, 0.2f, seq_idx);
            }
        }
        // 2. Process lidar_seq_X.bin
        else if (std::regex_match(filename, match, lidar_seq_regex)) {
            int seq_idx = std::stoi(match[1].str());
            // Keep legacy toggles for seq 0/1.
            if (seq_idx == 0 && !publish_converted_filtered_) continue;
            if (seq_idx == 1 && !publish_converted_raw_) continue;
            std::string topic_name = "lidar_seq_" + std::to_string(seq_idx);

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

}

void StaticMapLoader::publishPointsByPolylineId(
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

}  // namespace realtime_line_generator::viewer
