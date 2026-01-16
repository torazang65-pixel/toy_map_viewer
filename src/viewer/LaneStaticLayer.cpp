#include "viewer/LaneStaticLayer.h"

#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/bind.hpp>
#include <fstream>
#include <sys/stat.h>
#include <cmath>
#include <array>

namespace {
    bool fileExists(const std::string& name) {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    }

    uint32_t hashId(int id) {
        uint32_t x = static_cast<uint32_t>(id);
        x = x * 2654435761;
        x = ((x >> 16) ^ x) * 0x45d9f3b;
        x = ((x >> 16) ^ x) * 0x45d9f3b;
        x = (x >> 16) ^ x;
        return x;
    }

    std::array<float, 3> hsvToRgb(float h, float s, float v) {
        if (s == 0.0f) return {v, v, v};
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
}

LaneStaticLayer::LaneStaticLayer(const std::string& name, ros::NodeHandle& nh,
                                 interactive_markers::InteractiveMarkerServer& server,
                                 interactive_markers::MenuHandler& menu_handler)
    : StaticLayer(name),
      nh_(nh),
      server_(server),
      menu_handler_(menu_handler) {}

void LaneStaticLayer::initMenu() {
    menu_handle_lane_ = menu_handler_.insert(
        "Explicit Lane",
        boost::bind(&LaneStaticLayer::processLaneFeedback, this, _1));
    menu_handler_.setCheckState(menu_handle_lane_, interactive_markers::MenuHandler::UNCHECKED);
}

void LaneStaticLayer::clearMarkers() {
    visualization_msgs::MarkerArray clear_msg;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = 3;
    clear_marker.header.frame_id = "map";
    clear_msg.markers.push_back(clear_marker);
    for (auto& pub : lane_publishers_) pub.publish(clear_msg);
    ros::Duration(0.05).sleep();
}

void LaneStaticLayer::clear() {
    clearMarkers();
    server_.clear();
    server_.applyChanges();

    for (auto& pub : lane_publishers_) pub.shutdown();
    lane_publishers_.clear();
    lane_properties_.clear();
}

std_msgs::ColorRGBA LaneStaticLayer::generateColor(int id) {
    uint32_t hashed_id = hashId(id);
    double hue = std::fmod(static_cast<double>(hashed_id) * 0.61803398875, 1.0);
    if (0.62f <= hue && hue <= 0.7f) hue = std::fmod(hue + 0.2f, 1.0f);
    auto rgb = hsvToRgb(static_cast<float>(hue), 0.85f, 0.95f);

    std_msgs::ColorRGBA color;
    color.r = rgb[0];
    color.g = rgb[1];
    color.b = rgb[2];
    color.a = 1.0f;
    return color;
}

void LaneStaticLayer::updateMarkerVisual(const std::string& marker_name) {
    auto it = lane_properties_.find(marker_name);
    if (it == lane_properties_.end()) return;

    LaneProp& prop = it->second;
    visualization_msgs::InteractiveMarker& int_marker = prop.int_marker;
    if (!int_marker.controls.empty() && !int_marker.controls[0].markers.empty()) {
        visualization_msgs::Marker& p_marker = int_marker.controls[0].markers[0];
        p_marker.color = prop.original_color;
        if (prop.explicit_lane) {
            p_marker.scale.x = 0.5;
            p_marker.scale.y = 0.5;
            p_marker.scale.z = 0.5;
            p_marker.color.a = 1.0f;
        } else {
            p_marker.scale.x = 0.2;
            p_marker.scale.y = 0.2;
            p_marker.scale.z = 0.2;
            p_marker.color.a = 0.3f;
        }
    }
    server_.insert(int_marker);

    auto state = prop.explicit_lane ? interactive_markers::MenuHandler::CHECKED
                                    : interactive_markers::MenuHandler::UNCHECKED;
    menu_handler_.setCheckState(menu_handle_lane_, state);
    menu_handler_.apply(server_, marker_name);
}

void LaneStaticLayer::processLaneFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) return;

    std::string name = feedback->marker_name;
    auto it = lane_properties_.find(name);
    if (it != lane_properties_.end()) {
        it->second.explicit_lane = !it->second.explicit_lane;
        updateMarkerVisual(name);
        applyMenuState();
    }
}

void LaneStaticLayer::processFile(const std::string& path, ros::Publisher& pub, int seq_idx,
                                  double off_x, double off_y, double off_z) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return;

    uint32_t cluster_num = 0;
    ifs.read(reinterpret_cast<char*>(&cluster_num), 4);

    visualization_msgs::MarkerArray line_markers_msg;
    std::string seq_str = "seq_" + std::to_string(seq_idx);

    for (uint32_t i = 0; i < cluster_num; ++i) {
        int32_t id, layer;
        uint32_t point_num;
        bool explicit_lane = false;

        ifs.read(reinterpret_cast<char*>(&id), 4);
        ifs.read(reinterpret_cast<char*>(&layer), 4);
        ifs.read(reinterpret_cast<char*>(&explicit_lane), sizeof(bool));
        ifs.read(reinterpret_cast<char*>(&point_num), 4);

        std_msgs::ColorRGBA color = generateColor(id);
        std::vector<geometry_msgs::Point> lane_points;
        lane_points.reserve(point_num);

        for (uint32_t j = 0; j < point_num; ++j) {
            float x, y, z;
            ifs.read(reinterpret_cast<char*>(&x), 4);
            ifs.read(reinterpret_cast<char*>(&y), 4);
            ifs.read(reinterpret_cast<char*>(&z), 4);

            geometry_msgs::Point p;
            p.x = x - off_x;
            p.y = y - off_y;
            p.z = z - off_z;
            lane_points.push_back(p);
        }

        std::string marker_name = seq_str + "/" + std::to_string(id);
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.name = marker_name;

        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        control.always_visible = true;

        visualization_msgs::Marker point_marker;
        point_marker.type = visualization_msgs::Marker::CUBE_LIST;
        for (const auto& p : lane_points) point_marker.points.push_back(p);
        if (!point_marker.points.empty()) control.markers.push_back(point_marker);
        int_marker.controls.push_back(control);
        server_.insert(int_marker);

        LaneProp prop;
        prop.explicit_lane = explicit_lane;
        prop.original_color = color;
        prop.int_marker = int_marker;
        lane_properties_[marker_name] = prop;

        updateMarkerVisual(marker_name);

        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "lines_" + seq_str;
        line_marker.id = id;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = explicit_lane ? 0.3 : 0.1;
        line_marker.color = color;
        line_marker.color.a = explicit_lane ? 1.0 : 0.3;
        line_marker.pose.orientation.w = 1.0;

        for (size_t k = 0; k + 1 < lane_points.size(); ++k) {
            line_marker.points.push_back(lane_points[k]);
            line_marker.points.push_back(lane_points[k + 1]);
        }
        if (!line_marker.points.empty()) line_markers_msg.markers.push_back(line_marker);
    }

    pub.publish(line_markers_msg);
    applyMenuState();
}

void LaneStaticLayer::loadData(const std::string& base_dir, double off_x, double off_y, double off_z) {
    int seq_idx = 0;
    while (true) {
        std::string filename = "points_seq_" + std::to_string(seq_idx) + ".bin";
        std::string full_path = base_dir + filename;
        if (!fileExists(full_path)) break;

        std::string topic_name = "/lane_viz/seq_" + std::to_string(seq_idx);
        ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
        lane_publishers_.push_back(pub);
        processFile(full_path, pub, seq_idx, off_x, off_y, off_z);
        seq_idx++;
    }
    server_.applyChanges();
}

void LaneStaticLayer::applyMenuState() {
    for (const auto& [name, prop] : lane_properties_) {
        menu_handler_.apply(server_, name);
    }
    server_.applyChanges();
}
