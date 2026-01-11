#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <sys/stat.h>
#include <map>

// (색상 함수들 - Lane용)
static uint32_t hashId(int id) {
    uint32_t x = id; x = x * 2654435761; x = ((x >> 16) ^ x) * 0x45d9f3b; x = ((x >> 16) ^ x) * 0x45d9f3b; x = (x >> 16) ^ x; return x;
}
static std::array<float, 3> hsvToRgb(float h, float s, float v) {
    if (s == 0.0f) return {v, v, v}; int i = static_cast<int>(h * 6.0f); float f = (h * 6.0f) - i; float p = v * (1.0f - s); float q = v * (1.0f - s * f); float t = v * (1.0f - s * (1.0f - f)); i %= 6; switch (i) { case 0: return {v, t, p}; case 1: return {q, v, p}; case 2: return {p, v, t}; case 3: return {p, q, v}; case 4: return {t, p, v}; default: return {v, p, q}; }
}
static std::array<float, 3> generateColor(int id) {
    uint32_t hashed_id = hashId(id); double hue = std::fmod(static_cast<double>(hashed_id) * 0.61803398875, 1.0); if (0.62f <= hue && hue <= 0.7f) hue = std::fmod(hue + 0.2f, 1.0f); return hsvToRgb(static_cast<float>(hue), 0.85f, 0.95f);
}

class BinMapViewer {
    struct LaneProp {
        bool explicit_lane;
        std_msgs::ColorRGBA original_color;
        visualization_msgs::InteractiveMarker int_marker;
    };

public:
    BinMapViewer() 
      : server_("lane_points_server") 
    {
        ros::NodeHandle nh("~");
        
        std::string default_dir = ros::package::getPath("toy_map_viewer") + "/data/issue/converted_bin/";

        int file_idx;
        nh.param<int>("file_idx", file_idx, 20000);

        base_dir_ = default_dir + std::to_string(file_idx) + "/";

        ROS_INFO("Target Directory: %s", base_dir_.c_str());

        offset_x_ = 0.0; offset_y_ = 0.0; offset_z_ = 0.0;
        is_initialized_ = false;

        menu_handle_ = menu_handler_.insert("Explicit Lane", boost::bind(&BinMapViewer::processFeedback, this, _1));

        loadAllSequences(nh);
        loadLidarSequences(nh);
    }

    bool fileExists(const std::string& name) {
        struct stat buffer;   
        return (stat (name.c_str(), &buffer) == 0); 
    }

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
            std::string name = feedback->marker_name;
            if (lane_properties_.find(name) != lane_properties_.end()) {
                lane_properties_[name].explicit_lane = !lane_properties_[name].explicit_lane;
                bool is_explicit = lane_properties_[name].explicit_lane;
                updateMarkerVisual(name);
                interactive_markers::MenuHandler::CheckState state = 
                    is_explicit ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED;
                menu_handler_.setCheckState(menu_handle_, state);
                menu_handler_.apply(server_, name);
                server_.applyChanges();
            }
        }
    }

    void updateMarkerVisual(const std::string& marker_name) {
        if (lane_properties_.find(marker_name) == lane_properties_.end()) return;
        LaneProp& prop = lane_properties_[marker_name];
        visualization_msgs::InteractiveMarker& int_marker = prop.int_marker;

        if (!int_marker.controls.empty() && !int_marker.controls[0].markers.empty()) {
            visualization_msgs::Marker& p_marker = int_marker.controls[0].markers[0];
            p_marker.color = prop.original_color;
            if (prop.explicit_lane) {
                p_marker.scale.x = 0.5; p_marker.scale.y = 0.5; p_marker.scale.z = 0.5; p_marker.color.a = 1.0f; 
            } else {
                p_marker.scale.x = 0.2; p_marker.scale.y = 0.2; p_marker.scale.z = 0.2; p_marker.color.a = 0.3f;
            }
        }
        server_.insert(int_marker);
    }

    void loadAllSequences(ros::NodeHandle& nh) {
        int seq_idx = 0;
        while (true) {
            std::string filename = "points_seq_" + std::to_string(seq_idx) + ".bin";
            std::string full_path = base_dir_ + filename;

            if (!fileExists(full_path)) {
                if (seq_idx > 0) ROS_INFO("Finished loading sequence files up to index %d.", seq_idx - 1);
                else ROS_WARN("No sequence files found in %s", base_dir_.c_str());
                break;
            }
            ROS_INFO("Found file: %s", filename.c_str());

            std::string topic_name = "/lane_viz/seq_" + std::to_string(seq_idx);
            ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
            publishers_.push_back(pub);

            processFile(full_path, pub, seq_idx);
            seq_idx++;
        }
        server_.applyChanges(); 
    }

    void loadLidarSequences(ros::NodeHandle& nh) {
        int seq_idx = 0;
        while(true) {
            std::string filename = "lidar_seq_" + std::to_string(seq_idx) + ".bin";
            std::string full_path = base_dir_ + filename;

            if (!fileExists(full_path)) {
                if (seq_idx > 0) ROS_INFO("Finished loading Lidar sequence files up to index %d.", seq_idx - 1);
                else ROS_WARN("No Lidar sequence files found in %s", base_dir_.c_str());
                break;
            }
            ROS_INFO("Found Lidar file: %s", filename.c_str());

            std::string topic_name = "/lidar_viz/seq_" + std::to_string(seq_idx);
            ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
            publishers_.push_back(pub);

            processLidarFile(full_path, pub, seq_idx);
            seq_idx++;
        }
    }

    // Lane 데이터 처리
    void processFile(const std::string& path, ros::Publisher& pub, int seq_idx) {
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

            auto [r, g, b] = generateColor(id);
            std_msgs::ColorRGBA color;
            color.r = r; color.g = g; color.b = b; color.a = 1.0f;

            std::vector<geometry_msgs::Point> lane_points;
            lane_points.reserve(point_num);

            for (uint32_t j = 0; j < point_num; ++j) {
                float x, y, z;
                ifs.read(reinterpret_cast<char*>(&x), 4);
                ifs.read(reinterpret_cast<char*>(&y), 4);
                ifs.read(reinterpret_cast<char*>(&z), 4);
                
                if (!is_initialized_) {
                    offset_x_ = x; offset_y_ = y; offset_z_ = z;
                    is_initialized_ = true;
                }
                geometry_msgs::Point p;
                p.x = x - offset_x_; p.y = y - offset_y_; p.z = z - offset_z_;
                lane_points.push_back(p);
            }

            // Interactive Marker (Lane)
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
            interactive_markers::MenuHandler::CheckState state = 
                explicit_lane ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED;
            menu_handler_.setCheckState(menu_handle_, state);
            menu_handler_.apply(server_, marker_name);

            // Line Marker (Lane)
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
            for (size_t k = 0; k < lane_points.size(); ++k) {
                if (k + 1 < lane_points.size()) {
                    line_marker.points.push_back(lane_points[k]);
                    line_marker.points.push_back(lane_points[k+1]);
                }
            }
            if (!line_marker.points.empty()) line_markers_msg.markers.push_back(line_marker);
        }
        pub.publish(line_markers_msg);
    }

    // [수정됨] Lidar 파일 처리: x, y, z만 읽어서 단일 색상으로 표시
    void processLidarFile(const std::string& path, ros::Publisher& pub, int seq_idx) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) return;

        uint32_t cluster_num = 0;
        ifs.read(reinterpret_cast<char*>(&cluster_num), 4);
        
        visualization_msgs::MarkerArray msg;
        
        visualization_msgs::Marker big_marker;
        big_marker.header.frame_id = "map";
        big_marker.header.stamp = ros::Time::now();
        big_marker.ns = "lidar_merged_" + std::to_string(seq_idx);
        big_marker.id = 0;
        big_marker.type = visualization_msgs::Marker::POINTS;
        big_marker.action = visualization_msgs::Marker::ADD;
        
        big_marker.pose.orientation.w = 1.0;
        
        big_marker.scale.x = 0.05; // 점 크기
        big_marker.scale.y = 0.05;
        
        // 단일 색상 설정 (밝은 회색)
        big_marker.color.r = 0.9;
        big_marker.color.g = 0.9; 
        big_marker.color.b = 0.9; 
        big_marker.color.a = 0.8; 

        for (uint32_t i = 0; i < cluster_num; ++i) {
            int32_t id;
            uint32_t point_num;
            
            ifs.read(reinterpret_cast<char*>(&id), 4);
            ifs.read(reinterpret_cast<char*>(&point_num), 4);

            for (uint32_t j = 0; j < point_num; ++j) {
                float x, y, z;
                // [중요] x, y, z만 읽습니다 (BinSaver.h와 일치해야 함)
                ifs.read(reinterpret_cast<char*>(&x), 4);
                ifs.read(reinterpret_cast<char*>(&y), 4);
                ifs.read(reinterpret_cast<char*>(&z), 4);

                geometry_msgs::Point p;
                p.x = x - offset_x_; 
                p.y = y - offset_y_;
                p.z = z - offset_z_;
                
                big_marker.points.push_back(p);
            }
        }
        
        if (!big_marker.points.empty()) {
            msg.markers.push_back(big_marker);
            pub.publish(msg);
            ROS_INFO("Published merged lidar cloud with %lu points", big_marker.points.size());
        }
    }
    
    void spin() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    interactive_markers::InteractiveMarkerServer server_;
    
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle menu_handle_;

    std::vector<ros::Publisher> publishers_;
    std::string base_dir_;
    double offset_x_, offset_y_, offset_z_;
    bool is_initialized_;
    
    std::map<std::string, LaneProp> lane_properties_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bin_map_viewer");
    BinMapViewer viewer;
    viewer.spin(); 
    return 0;
}