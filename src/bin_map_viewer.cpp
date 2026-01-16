#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_msgs/Int32.h>

#include "viewer/StaticLayer.h"
#include "viewer/LaneStaticLayer.h"
#include "viewer/LidarStaticLayer.h"
#include "viewer/VehicleTrajectoryLayer.h"
#include "viewer/PointCloudLayer.h"
#include "viewer/LaneLayer.h"

#include <fstream>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <map>
#include <memory>

namespace {
    bool fileExists(const std::string& name) {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    }
}

// ==========================================
// [Main Class] 통합 뷰어
// ==========================================
class BinMapViewer {
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
        // 1. 정적 레이어 등록
        // ---------------------------------------------------------
        lane_static_layer_ = std::make_shared<LaneStaticLayer>("Static Lane", nh_, server_, menu_handler_);
        static_layers_.push_back(lane_static_layer_);
        static_layers_.push_back(std::make_shared<LidarStaticLayer>("Static Lidar", nh_));
        static_layers_.push_back(std::make_shared<VehicleTrajectoryLayer>("Vehicle Trajectory", nh_));

        // ---------------------------------------------------------
        // 2. 애니메이션 레이어 등록 (캡슐화 적용)
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

        // 3. 메뉴 초기화
        initMenu();

        // 4. Subscriber & Timer
        sub_idx_ = nh.subscribe("/change_map_idx", 1, &BinMapViewer::changeIdxCallback, this);
        play_timer_ = nh.createTimer(ros::Duration(0.1), &BinMapViewer::playCallback, this);

        // 5. 로드
        loadMap(start_file_idx);
    }

    void initMenu() {
        // [Static] Lane Explicit Toggle
        if (lane_static_layer_) lane_static_layer_->initMenu();

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
        for (auto& layer : static_layers_) layer->clear();

        is_initialized_ = false;
        offset_x_ = 0.0;
        offset_y_ = 0.0;
        offset_z_ = 0.0;
        current_frame_idx_ = 0;

        base_dir_ = default_dir_ + std::to_string(file_idx) + "/";
        ROS_INFO("Loading Map from: %s", base_dir_.c_str());

        // 2. 원점 기준 초기화 (가장 먼저 읽는 파일 기준)
        initOffsetFromFirstAvailableFile();

        // 3. 정적 맵 로드 (오프셋이 설정되어 있으면 그대로 사용)
        for (auto& layer : static_layers_) {
            layer->loadData(base_dir_, offset_x_, offset_y_, offset_z_);
        }

        // 4. 애니메이션 레이어 데이터 로드 (설정된 오프셋 전달)
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

    // 1. Play/Pause
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
        if (lane_static_layer_) lane_static_layer_->applyMenuState();
    }

    void spin() { ros::spin(); }

private:
    bool initOffsetFromFirstAvailableFile() {
        struct Candidate {
            std::string path;
            enum class Type { kLaneLike, kLidar } type;
        };

        std::vector<Candidate> candidates = {
            {base_dir_ + "points_seq_0.bin", Candidate::Type::kLaneLike},
            {base_dir_ + "lidar_seq_0.bin", Candidate::Type::kLidar},
            {base_dir_ + "vehicle_trajectory.bin", Candidate::Type::kLaneLike},
        };

        for (const auto& c : candidates) {
            if (!fileExists(c.path)) continue;
            double x = 0.0, y = 0.0, z = 0.0;
            bool ok = false;
            if (c.type == Candidate::Type::kLaneLike) {
                ok = readFirstLaneLikePoint(c.path, x, y, z);
            } else {
                ok = readFirstLidarPoint(c.path, x, y, z);
            }

            if (ok) {
                offset_x_ = x;
                offset_y_ = y;
                offset_z_ = z;
                is_initialized_ = true;
                ROS_INFO("Origin initialized from: %s (%.3f, %.3f, %.3f)",
                         c.path.c_str(), offset_x_, offset_y_, offset_z_);
                return true;
            }
        }
        return false;
    }

    bool readFirstLaneLikePoint(const std::string& path, double& x, double& y, double& z) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) return false;

        uint32_t cluster_num = 0;
        ifs.read(reinterpret_cast<char*>(&cluster_num), 4);
        if (cluster_num == 0) return false;

        int32_t id, layer;
        bool explicit_lane = false;
        uint32_t point_num = 0;
        ifs.read(reinterpret_cast<char*>(&id), 4);
        ifs.read(reinterpret_cast<char*>(&layer), 4);
        ifs.read(reinterpret_cast<char*>(&explicit_lane), sizeof(bool));
        ifs.read(reinterpret_cast<char*>(&point_num), 4);
        if (point_num == 0) return false;

        float xf = 0.0f, yf = 0.0f, zf = 0.0f;
        ifs.read(reinterpret_cast<char*>(&xf), 4);
        ifs.read(reinterpret_cast<char*>(&yf), 4);
        ifs.read(reinterpret_cast<char*>(&zf), 4);
        if (!ifs.good()) return false;

        x = static_cast<double>(xf);
        y = static_cast<double>(yf);
        z = static_cast<double>(zf);
        return true;
    }

    bool readFirstLidarPoint(const std::string& path, double& x, double& y, double& z) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) return false;

        uint32_t cluster_num = 0;
        ifs.read(reinterpret_cast<char*>(&cluster_num), sizeof(uint32_t));
        if (cluster_num == 0) return false;

        int32_t id = 0;
        uint32_t point_num = 0;
        ifs.read(reinterpret_cast<char*>(&id), sizeof(int32_t));
        ifs.read(reinterpret_cast<char*>(&point_num), sizeof(uint32_t));
        if (point_num == 0) return false;

        float buffer[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        ifs.read(reinterpret_cast<char*>(buffer), sizeof(float) * 4);
        if (!ifs.good()) return false;

        x = static_cast<double>(buffer[0]);
        y = static_cast<double>(buffer[1]);
        z = static_cast<double>(buffer[2]);
        return true;
    }

    ros::NodeHandle nh_;
    interactive_markers::InteractiveMarkerServer server_;
    ros::Subscriber sub_idx_;
    interactive_markers::MenuHandler menu_handler_;
    
    // 메뉴 핸들
    interactive_markers::MenuHandler::EntryHandle menu_handle_play_;
    std::map<interactive_markers::MenuHandler::EntryHandle, int> layer_menu_handles_;

    // Static Layers
    std::shared_ptr<LaneStaticLayer> lane_static_layer_;
    std::vector<std::shared_ptr<StaticLayer>> static_layers_;

    // Common
    std::string default_dir_;
    std::string base_dir_;
    double offset_x_, offset_y_, offset_z_;
    bool is_initialized_;

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
