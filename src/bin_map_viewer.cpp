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

#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <sys/stat.h>
#include <map>

// ... (기존 색상 함수들 hashId, hsvToRgb, generateColor 등은 그대로 유지) ...
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
        
        default_dir_ = ros::package::getPath("toy_map_viewer") + "/data/issue/converted_bin/";

        int start_file_idx;
        nh.param<int>("file_idx", start_file_idx, 20000);

        offset_x_ = 0.0; offset_y_ = 0.0; offset_z_ = 0.0;
        is_initialized_ = false;

        menu_handle_ = menu_handler_.insert("Explicit Lane", boost::bind(&BinMapViewer::processFeedback, this, _1));
        sub_idx_ = nh.subscribe("/change_map_idx", 1, &BinMapViewer::changeIdxCallback, this);

        // [추가] 애니메이션 Publisher 및 타이머 초기화
        anim_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pred_animation", 1);
        // 타이머는 loadMap 이후 데이터가 있을 때만 유효하게 동작함
        play_timer_ = nh_.createTimer(ros::Duration(1.0), &BinMapViewer::playCallback, this);

        // 최초 로드
        loadMap(start_file_idx);
    }

    void changeIdxCallback(const std_msgs::Int32::ConstPtr& msg) {
        ROS_INFO("Received request to change map index to: %d", msg->data);
        loadMap(msg->data);
    }

    void clearMarkers() {
        visualization_msgs::MarkerArray clear_msg;
        visualization_msgs::Marker clear_marker;
        clear_marker.action = 3; // DELETEALL
        clear_marker.header.frame_id = "map";
        clear_msg.markers.push_back(clear_marker);

        for (auto& pub : lane_publishers_){
            pub.publish(clear_msg);
        }
        ros::Duration(0.05).sleep();
    }

    void loadMap(int file_idx) {
        // 1. 기존 데이터 정리
        clearMarkers();
        server_.clear();
        server_.applyChanges();
        
        for(auto& pub : lane_publishers_) pub.shutdown();
        for(auto& pub : lidar_publishers_) pub.shutdown();
        lane_publishers_.clear();
        lidar_publishers_.clear();
        
        // 애니메이션 데이터 초기화
        animation_frames_.clear();
        current_frame_idx_ = 0;

        // 오프셋 초기화
        lane_properties_.clear();
        is_initialized_ = false; 
        offset_x_ = 0.0; offset_y_ = 0.0; offset_z_ = 0.0;

        // 2. 경로 재설정
        base_dir_ = default_dir_ + std::to_string(file_idx) + "/";
        ROS_INFO("Loading Map from: %s", base_dir_.c_str());

        // 3. 정적 맵 로드 (Lane, Lidar Global Map)
        loadAllSequences();
        loadLidarSequences();

        // 4. [추가] 애니메이션 프레임 로드
        loadAnimationData();
    }

    // [추가] 애니메이션용 개별 프레임 로드 함수
    void loadAnimationData() {
        // 파일 경로 패턴: pred_frames/frame_{idx}.bin
        int frame_idx = 0;
        
        // 오프셋이 초기화되지 않았다면 애니메이션 로딩을 미루거나 경고 (보통 loadLidarSequences에서 초기화됨)
        if (!is_initialized_) {
            ROS_WARN("Map offset not initialized. Animation frames might be misaligned.");
        }

        ROS_INFO(">>> Loading Animation Frames...");
        while (true) {
            std::string filename = "batch/batch_" + std::to_string(frame_idx) + ".bin";
            std::string full_path = base_dir_ + filename;

            if (!fileExists(full_path)) {
                break; // 파일이 없으면 루프 종료
            }

            // 파일 읽기
            std::ifstream ifs(full_path, std::ios::binary);
            if (!ifs.is_open()) break;

            // BinSaver 포맷 파싱: cluster_num -> id -> p_num -> points
            uint32_t cluster_num = 0;
            ifs.read(reinterpret_cast<char*>(&cluster_num), sizeof(uint32_t));

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

            // CoordinateConverter에서 저장할 때 cluster_num=1로 저장했음
            for (uint32_t i = 0; i < cluster_num; ++i) {
                int32_t id;
                uint32_t point_num;
                ifs.read(reinterpret_cast<char*>(&id), sizeof(int32_t));
                ifs.read(reinterpret_cast<char*>(&point_num), sizeof(uint32_t));

                for (uint32_t j = 0; j < point_num; ++j) {
                    float x, y, z, intensity;
                    ifs.read(reinterpret_cast<char*>(&x), sizeof(float));
                    ifs.read(reinterpret_cast<char*>(&y), sizeof(float));
                    ifs.read(reinterpret_cast<char*>(&z), sizeof(float));
                    ifs.read(reinterpret_cast<char*>(&intensity), sizeof(float));

                    pcl::PointXYZI pt;
                    // [중요] 정적 맵과 동일한 오프셋 적용
                    pt.x = x - offset_x_;
                    pt.y = y - offset_y_;
                    pt.z = z - offset_z_;
                    pt.intensity = intensity;
                    cloud->push_back(pt);
                }
            }
            ifs.close();

            animation_frames_.push_back(cloud);
            frame_idx++;
        }
        ROS_INFO("Loaded %lu frames for animation.", animation_frames_.size());
    }

    // [추가] 타이머 콜백: 프레임 재생
    void playCallback(const ros::TimerEvent&) {
        if (animation_frames_.empty()) return;

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_batch = animation_frames_[current_frame_idx_];

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*current_batch, output_msg);
        output_msg.header.frame_id = "map";
        output_msg.header.stamp = ros::Time::now();

        anim_pub_.publish(output_msg);

        current_frame_idx_ = (current_frame_idx_ + 1) % animation_frames_.size();
    }

    bool fileExists(const std::string& name) {
        struct stat buffer;   
        return (stat (name.c_str(), &buffer) == 0); 
    }

    // ... (processFeedback, updateMarkerVisual 함수 등은 기존 유지) ...
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

    void loadAllSequences() {
        int seq_idx = 0;
        while (true) {
            std::string filename = "points_seq_" + std::to_string(seq_idx) + ".bin";
            std::string full_path = base_dir_ + filename;

            if (!fileExists(full_path)) {
                if (seq_idx == 0) ROS_WARN("No Lane sequence files found in %s", base_dir_.c_str());
                break;
            }
            
            std::string topic_name = "/lane_viz/seq_" + std::to_string(seq_idx);
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
            lane_publishers_.push_back(pub);

            processFile(full_path, pub, seq_idx);
            seq_idx++;
        }
        server_.applyChanges(); 
    }

    void loadLidarSequences() {
        int seq_idx = 0;
        while(true) {
            std::string filename = "lidar_seq_" + std::to_string(seq_idx) + ".bin";
            std::string full_path = base_dir_ + filename;

            if (!fileExists(full_path)) {
                if (seq_idx == 0) ROS_WARN("No Lidar sequence files found in %s", base_dir_.c_str());
                break;
            }
            
            std::string topic_name = "/lidar_viz/seq_" + std::to_string(seq_idx);
            ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true);
            lidar_publishers_.push_back(pub);

            processLidarFile(full_path, pub);
            seq_idx++;
        }
    }

    void processFile(const std::string& path, ros::Publisher& pub, int seq_idx) {
        // ... (기존 Lane processFile 내용 동일: 파일 읽고 마커 생성) ...
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
            // ... (마커 생성 로직 생략, 기존 코드 그대로 사용) ...
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

    void processLidarFile(const std::string& path, ros::Publisher& pub) {
        // ... (기존 Lidar processFile 내용 동일) ...
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) return;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        uint32_t cluster_num = 0;
        ifs.read(reinterpret_cast<char*>(&cluster_num), sizeof(uint32_t));
        
        for (uint32_t i = 0; i < cluster_num; ++i) {
            int32_t id;
            uint32_t point_num;
            ifs.read(reinterpret_cast<char*>(&id), sizeof(int32_t));
            ifs.read(reinterpret_cast<char*>(&point_num), sizeof(uint32_t));

            for (uint32_t j = 0; j < point_num; ++j) {
                float buffer[4]; // x, y, z, theta를 한꺼번에 읽기 위한 버퍼
                ifs.read(reinterpret_cast<char*>(buffer), sizeof(float) * 4);

                if (!is_initialized_) {
                    offset_x_ = buffer[0]; offset_y_ = buffer[1]; offset_z_ = buffer[2];
                    is_initialized_ = true;
                }

                pcl::PointXYZI pt;
                pt.x = buffer[0] - offset_x_;
                pt.y = buffer[1] - offset_y_;
                pt.z = buffer[2] - offset_z_;
                
                // 2. 4번째 값(theta)을 intensity에 할당
                float theta = buffer[3];
                // 필요 시 정규화 로직 추가 (0 ~ PI)
                while (theta < 0) theta += 2 * M_PI;
                while (theta >= M_PI) theta -= M_PI;
                pt.intensity = theta; 

                cloud->push_back(pt);
            }
        }
        
        if (cloud->empty()) return;

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header.frame_id = "map";
        output_msg.header.stamp = ros::Time::now();
        pub.publish(output_msg);
    }

    void spin() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    interactive_markers::InteractiveMarkerServer server_;
    ros::Subscriber sub_idx_;
    
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle menu_handle_;

    std::vector<ros::Publisher> lane_publishers_;
    std::vector<ros::Publisher> lidar_publishers_;
    std::string default_dir_;
    std::string base_dir_;
    double offset_x_, offset_y_, offset_z_;
    bool is_initialized_;
    
    std::map<std::string, LaneProp> lane_properties_;

    // [추가] 애니메이션 관련 변수
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> animation_frames_;
    ros::Publisher anim_pub_;
    ros::Timer play_timer_;
    size_t current_frame_idx_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bin_map_viewer");
    BinMapViewer viewer;
    viewer.spin(); 
    return 0;
}