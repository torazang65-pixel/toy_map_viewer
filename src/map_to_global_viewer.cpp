#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <sys/stat.h>
#include <map>
#include <filesystem>

namespace fs = std::filesystem;

struct Point6D {
    double x, y, z;
    double dx, dy, dz;
};

struct Lane {
    int id;
    int type;
    bool valid;
    std::vector<Point6D> points;
};

// ID 기반 색상 생성 유틸리티
static uint32_t hashId(int id) {
    uint32_t x = id; x = x * 2654435761; x = ((x >> 16) ^ x) * 0x45d9f3b; x = ((x >> 16) ^ x) * 0x45d9f3b; x = (x >> 16) ^ x; return x;
}
static std::array<float, 3> hsvToRgb(float h, float s, float v) {
    if (s == 0.0f) return {v, v, v}; int i = static_cast<int>(h * 6.0f); float f = (h * 6.0f) - i; float p = v * (1.0f - s); float q = v * (1.0f - s * f); float t = v * (1.0f - s * (1.0f - f)); i %= 6; switch (i) { case 0: return {v, t, p}; case 1: return {q, v, p}; case 2: return {p, v, t}; case 3: return {p, q, v}; case 4: return {t, p, v}; default: return {v, p, q}; }
}
static std::array<float, 3> generateColor(int id) {
    uint32_t hashed_id = hashId(id); double hue = std::fmod(static_cast<double>(hashed_id) * 0.61803398875, 1.0); return hsvToRgb(static_cast<float>(hue), 0.85f, 0.95f);
}

class MapToGlobalViewer {
public:
    MapToGlobalViewer() 
        : nh_("~"), 
          latest_server_("latest_interactive_server"), 
          prev_server_("prev_interactive_server") 
    {
        nh_.param<std::string>("package_name", package_name_, "toy_map_viewer");
        nh_.param<std::string>("date", date, "2025-09-26-14-21-28_maxen_v6_2");

        std::string pkg_path = ros::package::getPath(package_name_);
        base_dir_ = pkg_path + "/data/lane_change_data_converted/";
        frame_id_file_ = pkg_path + "/data/lane_change_data/Raw/" + date.c_str() + "/zone_info";

        std::ifstream zone_ifs(frame_id_file_);
        if (zone_ifs.is_open()) {
            zone_ifs >> frame_id_; // 파일의 첫 단어(frame_id)를 읽음
            ROS_INFO(">>> [%s] Detected frame_id: %s", date.c_str(), frame_id_.c_str());
            zone_ifs.close();
        } else {
            ROS_WARN(">>> [%s] zone_info not found at %s. Using default: 'world'", 
                    date.c_str(), frame_id_file_.c_str());
        }

        offset_x_ = 0.0; offset_y_ = 0.0; offset_z_ = 0.0;
        is_initialized_ = false;

        loadAndPublishAll();
    }

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {
            ROS_INFO("Selected: %s", feedback->marker_name.c_str());
        }
    }

    void loadAndPublishAll() {
        latest_server_.clear();
        prev_server_.clear();

        latest_dir_ = base_dir_ + "map_latest/" + date + "/";
        prev_dir_ = base_dir_ + "map_prev/" + date + "/";

        if (!latest_dir_.empty()) {
            ROS_INFO("Processing Latest Directory...");
            ROS_INFO("%s", latest_dir_.c_str());
            processDirectory(latest_dir_, "latest", 1.0f, 0.3f, latest_server_);
        }
        if (!prev_dir_.empty()) {
            ROS_INFO("Processing Prev Directory...");
            ROS_INFO("%s", prev_dir_.c_str());
            processDirectory(prev_dir_, "prev", 0.5f, 0.2f, prev_server_);
        }

        latest_server_.applyChanges();
        prev_server_.applyChanges();
    }

private:
    void processDirectory(const std::string& dir_path, const std::string& ns_prefix, 
                          float alpha, float scale, 
                          interactive_markers::InteractiveMarkerServer& server) 
    {
        if (!fs::exists(dir_path)) return;

        for (const auto& entry : fs::directory_iterator(dir_path)) {
            if (entry.path().extension() == ".bin") {
                std::string file_name = entry.path().stem().string();
                
                // 파일별 고유 토픽 생성 (예: /latest/points_seq_0)
                std::string topic_name = "/" + ns_prefix + "/" + file_name;
                if (file_pubs_.find(topic_name) == file_pubs_.end()) {
                    file_pubs_[topic_name] = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
                }

                visualization_msgs::MarkerArray marker_array;
                std::ifstream ifs(entry.path().string(), std::ios::binary);
                if (!ifs.is_open()) continue;

                uint32_t cluster_num = 0;
                ifs.read(reinterpret_cast<char*>(&cluster_num), sizeof(uint32_t));

                for (uint32_t i = 0; i < cluster_num; ++i) {
                    int32_t id, layer;
                    uint32_t p_num;
                    ifs.read(reinterpret_cast<char*>(&id), sizeof(int32_t));
                    ifs.read(reinterpret_cast<char*>(&layer), sizeof(int32_t));
                    ifs.read(reinterpret_cast<char*>(&p_num), sizeof(uint32_t));

                    auto [r, g, b] = generateColor(id);
                    std::vector<geometry_msgs::Point> points;

                    for (uint32_t j = 0; j < p_num; ++j) {
                        double x, y, z;
                        ifs.read(reinterpret_cast<char*>(&x), sizeof(double));
                        ifs.read(reinterpret_cast<char*>(&y), sizeof(double));
                        ifs.read(reinterpret_cast<char*>(&z), sizeof(double));

                        if (!is_initialized_) {

                            ROS_INFO("%f, %f, %f", x, y, z);
                            offset_x_ = 0;
                            offset_y_ = 0;
                            offset_z_ = 0;
                            is_initialized_ = true;
                            ROS_INFO("Map Origin Set (Double Precision): %.2f, %.2f", offset_x_, offset_y_);
                        }

                        geometry_msgs::Point p;
                        p.x = x - offset_x_; 
                        p.y = y - offset_y_; 
                        p.z = z - offset_z_;
                        points.push_back(p);
                    }

                    if (points.empty()) continue;

                    // 1. MarkerArray용 점(Sphere) 마커 - 토픽별로 관리됨
                    visualization_msgs::Marker p_marker;
                    p_marker.header.frame_id = frame_id_;
                    p_marker.header.stamp = ros::Time::now();
                    p_marker.ns = file_name;
                    p_marker.id = id;
                    p_marker.type = visualization_msgs::Marker::SPHERE_LIST;
                    p_marker.action = visualization_msgs::Marker::ADD;
                    p_marker.scale.x = scale; p_marker.scale.y = scale; p_marker.scale.z = scale;
                    p_marker.color.r = r; p_marker.color.g = g; p_marker.color.b = b; p_marker.color.a = alpha;
                    p_marker.points = points;

                    p_marker.pose.orientation.x = 0.0;
                    p_marker.pose.orientation.y = 0.0;
                    p_marker.pose.orientation.z = 0.0;
                    p_marker.pose.orientation.w = 1.0;

                    marker_array.markers.push_back(p_marker);

                    // 2. Interactive Marker 설정 - 클릭/피드백용
                    visualization_msgs::InteractiveMarker int_marker;
                    int_marker.header.frame_id = frame_id_;
                    int_marker.name = ns_prefix + "_" + file_name + "_ID_" + std::to_string(id);
                    int_marker.pose.orientation.w = 1.0;

                    visualization_msgs::InteractiveMarkerControl control;
                    control.always_visible = true;
                    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
                    
                    // 시각적 표현은 MarkerArray와 동일하게 SPHERE_LIST 사용
                    control.markers.push_back(p_marker); 
                    int_marker.controls.push_back(control);

                    server.insert(int_marker, boost::bind(&MapToGlobalViewer::processFeedback, this, _1));
                }
                
                // 파일별 토픽 발행
                file_pubs_[topic_name].publish(marker_array);
                ROS_INFO("Published %s with %u clusters.", topic_name.c_str(), cluster_num);
            }
        }
    }

    ros::NodeHandle nh_;
    interactive_markers::InteractiveMarkerServer latest_server_;
    interactive_markers::InteractiveMarkerServer prev_server_;
    std::map<std::string, ros::Publisher> file_pubs_;
    
    std::string date, package_name_, base_dir_, latest_dir_, prev_dir_, frame_id_file_, frame_id_;
    double offset_x_, offset_y_, offset_z_;
    bool is_initialized_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_to_global_viewer");
    MapToGlobalViewer viewer;
    ros::spin();
    return 0;
}