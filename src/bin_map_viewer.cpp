#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "viewer/PointCloudLayer.h"
#include "viewer/LaneLayer.h"

#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <sys/stat.h>
#include <map>
#include <memory>

// ==========================================
// [Utils] 색상 및 파일 유틸리티
// ==========================================
namespace Utils {
    static uint32_t hashId(int id) {
        uint32_t x = id; x = x * 2654435761; x = ((x >> 16) ^ x) * 0x45d9f3b; x = ((x >> 16) ^ x) * 0x45d9f3b; x = (x >> 16) ^ x; return x;
    }
    static std::array<float, 3> hsvToRgb(float h, float s, float v) {
        if (s == 0.0f) return {v, v, v}; int i = static_cast<int>(h * 6.0f); float f = (h * 6.0f) - i; float p = v * (1.0f - s); float q = v * (1.0f - s * f); float t = v * (1.0f - s * (1.0f - f)); i %= 6; switch (i) { case 0: return {v, t, p}; case 1: return {q, v, p}; case 2: return {p, v, t}; case 3: return {p, q, v}; case 4: return {t, p, v}; default: return {v, p, q}; }
    }
    static std::array<float, 3> generateColor(int id) {
        uint32_t hashed_id = hashId(id); double hue = std::fmod(static_cast<double>(hashed_id) * 0.61803398875, 1.0); if (0.62f <= hue && hue <= 0.7f) hue = std::fmod(hue + 0.2f, 1.0f); return hsvToRgb(static_cast<float>(hue), 0.85f, 0.95f);
    }
    static bool fileExists(const std::string& name) {
        struct stat buffer; return (stat (name.c_str(), &buffer) == 0); 
    }
}

// ==========================================
// [Main Class] 통합 뷰어
// ==========================================
class BinMapViewer {
    struct LaneProp {
        bool explicit_lane;
        std_msgs::ColorRGBA original_color;
        visualization_msgs::InteractiveMarker int_marker;
    };

public:
    BinMapViewer() : server_("lane_points_server") {
        ros::NodeHandle nh("~");
        
        default_dir_ = ros::package::getPath("toy_map_viewer") + "/data/issue/converted_bin/";
        int start_file_idx;
        nh.param<int>("file_idx", start_file_idx, 20000);

        offset_x_ = 0.0; offset_y_ = 0.0; offset_z_ = 0.0;
        is_initialized_ = false;
        is_playing_ = true;

        // ---------------------------------------------------------
        // 1. 애니메이션 레이어 등록 (캡슐화 적용)
        // ---------------------------------------------------------
        // (1) Batch Layer (Raw Point Cloud)
        layers_.push_back(std::make_shared<PointCloudLayer>(
            "Show Batch (Raw)", nh, "/pred_animation", "batch", "batch", false, false));
            
        // (2) Voxel Layer (Filtered Point Cloud)
        layers_.push_back(std::make_shared<PointCloudLayer>(
            "Show Voxel (Filtered)", nh, "/voxel_animation", "voxel", "voxel", true, false));

        // (3)
        layers_.push_back(std::make_shared<LaneLayer>("Show Generated Lane", nh, "/lane_animation", "lanes", "lane"));

        // (4)
        layers_.push_back(std::make_shared<LaneLayer>("Show Merged Lane", nh, "/merged_lane_animation", "merged_lanes", "lane"));

        // 2. 메뉴 초기화
        initMenu();

        // 3. Subscriber & Timer
        sub_idx_ = nh.subscribe("/change_map_idx", 1, &BinMapViewer::changeIdxCallback, this);
        play_timer_ = nh.createTimer(ros::Duration(1), &BinMapViewer::playCallback, this);

        // 4. 로드
        loadMap(start_file_idx);
    }

    void initMenu() {
        // [Static] Lane Explicit Toggle
        menu_handle_lane_ = menu_handler_.insert("Explicit Lane", boost::bind(&BinMapViewer::processLaneFeedback, this, _1));
        
        // [Animation] Play/Pause
        menu_handle_play_ = menu_handler_.insert("Pause Animation", boost::bind(&BinMapViewer::processAnimFeedback, this, _1));
        menu_handler_.setCheckState(menu_handle_play_, interactive_markers::MenuHandler::UNCHECKED);

        // [Animation] Layer Visibility (동적 생성)
        for (size_t i = 0; i < layers_.size(); ++i) {
            auto handle = menu_handler_.insert(layers_[i]->getName(), boost::bind(&BinMapViewer::processLayerFeedback, this, _1));
            menu_handler_.setCheckState(handle, interactive_markers::MenuHandler::CHECKED);
            layer_menu_handles_[handle] = i; // 메뉴 핸들 ID와 레이어 인덱스 매핑
        }
    }

    void changeIdxCallback(const std_msgs::Int32::ConstPtr& msg) {
        ROS_INFO("Received request to change map index to: %d", msg->data);
        loadMap(msg->data);
    }

    void loadMap(int file_idx) {
        // 1. 초기화
        clearMarkers();
        server_.clear();
        server_.applyChanges();
        
        for(auto& pub : lane_publishers_) pub.shutdown();
        for(auto& pub : lidar_publishers_) pub.shutdown();
        if(vehicle_trajectory_publisher_) vehicle_trajectory_publisher_.shutdown();
        lane_publishers_.clear();
        lidar_publishers_.clear();
        lane_properties_.clear();

        is_initialized_ = false;
        current_frame_idx_ = 0;

        base_dir_ = default_dir_ + std::to_string(file_idx) + "/";
        ROS_INFO("Loading Map from: %s", base_dir_.c_str());

        // 2. 정적 맵 로드 (여기서 offset_x_, y, z가 설정됨)
        loadAllSequences();
        loadLidarSequences();
        loadVehicleTrajectory();

        // 3. 애니메이션 레이어 데이터 로드 (설정된 오프셋 전달)
        for (auto& layer : layers_) {
            layer->loadData(base_dir_, offset_x_, offset_y_, offset_z_);
        }
    }

    // 타이머 콜백: 각 레이어의 publish 호출
    void playCallback(const ros::TimerEvent&) {
        if (!is_playing_) return;

        std_msgs::Header header;
        header.frame_id = "map";
        header.stamp = ros::Time::now();

        // 모든 레이어 Publish
        bool any_data = false;
        for (auto& layer : layers_) {
            if(layer->getFrameCount() > 0) any_data = true;
            layer->publish(current_frame_idx_, header);
        }

        if (any_data) current_frame_idx_++;
    }

    // --- Menu Callbacks ---

    // 1. Lane Toggle
    void processLaneFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) return;
        
        std::string name = feedback->marker_name;
        if (lane_properties_.find(name) != lane_properties_.end()) {
            lane_properties_[name].explicit_lane = !lane_properties_[name].explicit_lane;
            updateMarkerVisual(name); // 내부에서 apply 호출
            applyMenuState();
        }
    }

    // 2. Play/Pause
    void processAnimFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) return;
        
        is_playing_ = !is_playing_;
        if (is_playing_) ROS_INFO("Animation Resumed."); else ROS_INFO("Animation Paused.");
        applyMenuState();
    }

    // 3. Layer Visibility (Batch, Voxel, ...)
    void processLayerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) return;

        auto it = layer_menu_handles_.find(feedback->menu_entry_id);
        if (it != layer_menu_handles_.end()) {
            int layer_idx = it->second;
            bool current_vis = layers_[layer_idx]->isVisible();
            layers_[layer_idx]->setVisible(!current_vis); // Toggle
            
            ROS_INFO("Toggled visibility for layer: %s -> %s", 
                     layers_[layer_idx]->getName().c_str(), (!current_vis ? "ON" : "OFF"));
            
            applyMenuState();
        }
    }

    // 모든 마커에 현재 메뉴 상태(체크박스) 일괄 적용
    void applyMenuState() {
        // Play/Pause State
        auto play_state = !is_playing_ ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED;
        menu_handler_.setCheckState(menu_handle_play_, play_state);

        // Layer States
        for (auto const& [handle, idx] : layer_menu_handles_) {
            auto state = layers_[idx]->isVisible() ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED;
            menu_handler_.setCheckState(handle, state);
        }

        // 전체 마커에 적용
        for (auto const& [name, prop] : lane_properties_) {
            menu_handler_.apply(server_, name);
        }
        server_.applyChanges();
    }

    // ==========================================
    // [Static Map Loader] 기존 로직 유지
    // ==========================================
    
    void clearMarkers() {
        visualization_msgs::MarkerArray clear_msg;
        visualization_msgs::Marker clear_marker;
        clear_marker.action = 3; clear_marker.header.frame_id = "map";
        clear_msg.markers.push_back(clear_marker);
        for (auto& pub : lane_publishers_) pub.publish(clear_msg);
        ros::Duration(0.05).sleep();
    }

    void updateMarkerVisual(const std::string& marker_name) {
        if (lane_properties_.find(marker_name) == lane_properties_.end()) return;
        LaneProp& prop = lane_properties_[marker_name];
        visualization_msgs::InteractiveMarker& int_marker = prop.int_marker;
        if (!int_marker.controls.empty() && !int_marker.controls[0].markers.empty()) {
            visualization_msgs::Marker& p_marker = int_marker.controls[0].markers[0];
            p_marker.color = prop.original_color;
            if (prop.explicit_lane) { p_marker.scale.x=0.5; p_marker.scale.y=0.5; p_marker.scale.z=0.5; p_marker.color.a=1.0f; }
            else { p_marker.scale.x=0.2; p_marker.scale.y=0.2; p_marker.scale.z=0.2; p_marker.color.a=0.3f; }
        }
        server_.insert(int_marker);
        
        // 개별 마커 업데이트 시에도 메뉴 상태 동기화
        auto state = prop.explicit_lane ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED;
        menu_handler_.setCheckState(menu_handle_lane_, state);
        menu_handler_.apply(server_, marker_name);
    }
    
    void processFile(const std::string& path, ros::Publisher& pub, int seq_idx) {
        std::ifstream ifs(path, std::ios::binary); if (!ifs.is_open()) return;
        uint32_t cluster_num = 0; ifs.read(reinterpret_cast<char*>(&cluster_num), 4);
        visualization_msgs::MarkerArray line_markers_msg; std::string seq_str = "seq_" + std::to_string(seq_idx); 
        for (uint32_t i = 0; i < cluster_num; ++i) {
            int32_t id, layer; uint32_t point_num; bool explicit_lane = false;
            ifs.read(reinterpret_cast<char*>(&id), 4); ifs.read(reinterpret_cast<char*>(&layer), 4); ifs.read(reinterpret_cast<char*>(&explicit_lane), sizeof(bool)); ifs.read(reinterpret_cast<char*>(&point_num), 4);
            auto [r, g, b] = Utils::generateColor(id); std_msgs::ColorRGBA color; color.r = r; color.g = g; color.b = b; color.a = 1.0f;
            std::vector<geometry_msgs::Point> lane_points; lane_points.reserve(point_num);
            for (uint32_t j = 0; j < point_num; ++j) {
                float x, y, z; ifs.read(reinterpret_cast<char*>(&x), 4); ifs.read(reinterpret_cast<char*>(&y), 4); ifs.read(reinterpret_cast<char*>(&z), 4);
                if (!is_initialized_) { offset_x_ = x; offset_y_ = y; offset_z_ = z; is_initialized_ = true; }
                geometry_msgs::Point p; p.x = x - offset_x_; p.y = y - offset_y_; p.z = z - offset_z_; lane_points.push_back(p);
            }
            std::string marker_name = seq_str + "/" + std::to_string(id); 
            visualization_msgs::InteractiveMarker int_marker; int_marker.header.frame_id = "map"; int_marker.name = marker_name; 
            visualization_msgs::InteractiveMarkerControl control; control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON; control.always_visible = true;
            visualization_msgs::Marker point_marker; point_marker.type = visualization_msgs::Marker::CUBE_LIST; for (const auto& p : lane_points) point_marker.points.push_back(p);
            if (!point_marker.points.empty()) control.markers.push_back(point_marker); int_marker.controls.push_back(control); server_.insert(int_marker); 
            
            LaneProp prop; prop.explicit_lane = explicit_lane; prop.original_color = color; prop.int_marker = int_marker; lane_properties_[marker_name] = prop;
            
            // 시각화 업데이트
            updateMarkerVisual(marker_name);

            visualization_msgs::Marker line_marker; line_marker.header.frame_id = "map"; line_marker.header.stamp = ros::Time::now(); line_marker.ns = "lines_" + seq_str; line_marker.id = id; line_marker.type = visualization_msgs::Marker::LINE_LIST; line_marker.action = visualization_msgs::Marker::ADD; line_marker.scale.x = explicit_lane ? 0.3 : 0.1; line_marker.color = color; line_marker.color.a = explicit_lane ? 1.0 : 0.3; line_marker.pose.orientation.w = 1.0;
            for (size_t k = 0; k < lane_points.size(); ++k) { if (k + 1 < lane_points.size()) { line_marker.points.push_back(lane_points[k]); line_marker.points.push_back(lane_points[k+1]); } }
            if (!line_marker.points.empty()) line_markers_msg.markers.push_back(line_marker);
        }
        pub.publish(line_markers_msg);
        
        // 새로 로드된 마커들에 대해 현재 메뉴 상태 적용
        applyMenuState();
    }

    void processVehicleTrajectory(const std::string& path, ros::Publisher& pub) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) {
            ROS_WARN("Vehicle trajectory file not found: %s", path.c_str());
            return;
        }

        uint32_t cluster_num = 0;
        ifs.read(reinterpret_cast<char*>(&cluster_num), 4);

        visualization_msgs::MarkerArray marker_array;
        std::vector<geometry_msgs::Point> trajectory_points;

        for (uint32_t i = 0; i < cluster_num; ++i) {
            int32_t id, layer;
            bool explicit_lane = false;
            uint32_t point_num;

            ifs.read(reinterpret_cast<char*>(&id), 4);
            ifs.read(reinterpret_cast<char*>(&layer), 4);
            ifs.read(reinterpret_cast<char*>(&explicit_lane), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&point_num), 4);

            for (uint32_t j = 0; j < point_num; ++j) {
                float x, y, z;
                ifs.read(reinterpret_cast<char*>(&x), 4);
                ifs.read(reinterpret_cast<char*>(&y), 4);
                ifs.read(reinterpret_cast<char*>(&z), 4);

                if (!is_initialized_) {
                    offset_x_ = x;
                    offset_y_ = y;
                    offset_z_ = z;
                    is_initialized_ = true;
                }

                geometry_msgs::Point pt;
                pt.x = x - offset_x_;
                pt.y = y - offset_y_;
                pt.z = z - offset_z_;
                trajectory_points.push_back(pt);

                // Create sphere marker for each vehicle position
                visualization_msgs::Marker sphere_marker;
                sphere_marker.header.frame_id = "map";
                sphere_marker.header.stamp = ros::Time::now();
                sphere_marker.ns = "vehicle_positions";
                sphere_marker.id = j;
                sphere_marker.type = visualization_msgs::Marker::SPHERE;
                sphere_marker.action = visualization_msgs::Marker::ADD;

                sphere_marker.pose.position = pt;
                sphere_marker.pose.orientation.w = 1.0;

                sphere_marker.scale.x = 1.0;
                sphere_marker.scale.y = 1.0;
                sphere_marker.scale.z = 1.0;

                sphere_marker.color.r = 1.0;
                sphere_marker.color.g = 0.0;
                sphere_marker.color.b = 0.0;
                sphere_marker.color.a = 0.8;

                marker_array.markers.push_back(sphere_marker);
            }
        }

        // Add line strip to connect trajectory points
        if (trajectory_points.size() > 1) {
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "vehicle_trajectory_line";
            line_marker.id = 0;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;

            line_marker.pose.orientation.w = 1.0;
            line_marker.scale.x = 0.3;  // Line width

            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 0.8;

            line_marker.points = trajectory_points;

            marker_array.markers.push_back(line_marker);
        }

        pub.publish(marker_array);
        ROS_INFO("Loaded vehicle trajectory with %lu poses", trajectory_points.size());
    }

    void loadAllSequences() { int seq_idx = 0; while (true) { std::string filename = "points_seq_" + std::to_string(seq_idx) + ".bin"; std::string full_path = base_dir_ + filename; if (!Utils::fileExists(full_path)) break; std::string topic_name = "/lane_viz/seq_" + std::to_string(seq_idx); ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true); lane_publishers_.push_back(pub); processFile(full_path, pub, seq_idx); seq_idx++; } server_.applyChanges(); }
    void loadLidarSequences() { int seq_idx = 0; while(true) { std::string filename = "lidar_seq_" + std::to_string(seq_idx) + ".bin"; std::string full_path = base_dir_ + filename; if (!Utils::fileExists(full_path)) break; std::string topic_name = "/lidar_viz/seq_" + std::to_string(seq_idx); ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true); lidar_publishers_.push_back(pub); processLidarFile(full_path, pub); seq_idx++; } }
    void loadVehicleTrajectory() {
        std::string filename = "vehicle_trajectory.bin";
        std::string full_path = base_dir_ + filename;
        if (!Utils::fileExists(full_path)) {
            ROS_INFO("No vehicle trajectory file found");
            return;
        }
        vehicle_trajectory_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/vehicle_trajectory_viz", 1, true);
        processVehicleTrajectory(full_path, vehicle_trajectory_publisher_);
    }
    void processLidarFile(const std::string& path, ros::Publisher& pub) { 
        std::ifstream ifs(path, std::ios::binary); if (!ifs.is_open()) return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); uint32_t cluster_num = 0; ifs.read(reinterpret_cast<char*>(&cluster_num), sizeof(uint32_t));
        for (uint32_t i = 0; i < cluster_num; ++i) { int32_t id; uint32_t point_num; ifs.read(reinterpret_cast<char*>(&id), sizeof(int32_t)); ifs.read(reinterpret_cast<char*>(&point_num), sizeof(uint32_t));
            for (uint32_t j = 0; j < point_num; ++j) { float buffer[4]; ifs.read(reinterpret_cast<char*>(buffer), sizeof(float) * 4);
                if (!is_initialized_) { offset_x_ = buffer[0]; offset_y_ = buffer[1]; offset_z_ = buffer[2]; is_initialized_ = true; }
                pcl::PointXYZI pt; pt.x = buffer[0] - offset_x_; pt.y = buffer[1] - offset_y_; pt.z = buffer[2] - offset_z_; float theta = buffer[3]; while (theta < 0) theta += 2 * M_PI; while (theta >= M_PI) theta -= M_PI; pt.intensity = theta; cloud->push_back(pt); } }
        if (cloud->empty()) return; sensor_msgs::PointCloud2 output_msg; pcl::toROSMsg(*cloud, output_msg); output_msg.header.frame_id = "map"; output_msg.header.stamp = ros::Time::now(); pub.publish(output_msg);
    }
    void spin() { ros::spin(); }

private:
    ros::NodeHandle nh_;
    interactive_markers::InteractiveMarkerServer server_;
    ros::Subscriber sub_idx_;
    interactive_markers::MenuHandler menu_handler_;
    
    // 메뉴 핸들
    interactive_markers::MenuHandler::EntryHandle menu_handle_lane_;
    interactive_markers::MenuHandler::EntryHandle menu_handle_play_;
    std::map<interactive_markers::MenuHandler::EntryHandle, int> layer_menu_handles_;

    // Static Data
    std::vector<ros::Publisher> lane_publishers_;
    std::vector<ros::Publisher> lidar_publishers_;
    ros::Publisher vehicle_trajectory_publisher_;
    std::string default_dir_;
    std::string base_dir_;
    double offset_x_, offset_y_, offset_z_;
    bool is_initialized_;
    std::map<std::string, LaneProp> lane_properties_;

    // Animation Layers (Polymorphism)
    std::vector<std::shared_ptr<AnimationLayer>> layers_;
    
    ros::Timer play_timer_;
    size_t current_frame_idx_;
    bool is_playing_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bin_map_viewer");
    BinMapViewer viewer;
    viewer.spin(); 
    return 0;
}