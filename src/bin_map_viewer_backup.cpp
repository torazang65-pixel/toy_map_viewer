#include <ros/ros.h>
#include <ros/package.h> // 패키지 경로 찾기용
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <sys/stat.h> // 파일 존재 여부 확인용

// (색상 함수들 - 기존과 동일)
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
public:
    BinMapViewer() 
      : server_("lane_points_server") 
    {
        ros::NodeHandle nh("~");
        
        // 1. 기본 경로 설정 (파라미터가 없으면 패키지 내부 data 폴더 사용)
        std::string default_dir = ros::package::getPath("toy_map_viewer") + "/data/issue/converted_bin/";
        nh.param<std::string>("output_dir", base_dir_, default_dir);

        // 경로 끝에 '/'가 없으면 추가
        if (base_dir_.back() != '/') base_dir_ += "/";

        ROS_INFO("Target Directory: %s", base_dir_.c_str());

        offset_x_ = 0.0; offset_y_ = 0.0; offset_z_ = 0.0;
        is_initialized_ = false;

        // 2. 파일 자동 탐색 및 로딩
        loadAllSequences(nh);
    }

    // 파일 존재 여부 확인 헬퍼 함수
    bool fileExists(const std::string& name) {
        struct stat buffer;   
        return (stat (name.c_str(), &buffer) == 0); 
    }

    void loadAllSequences(ros::NodeHandle& nh) {
        int seq_idx = 0;
        while (true) {
            // 파일명 생성: points_seq_0.bin, points_seq_1.bin ...
            std::string filename = "points_seq_" + std::to_string(seq_idx) + ".bin";
            std::string full_path = base_dir_ + filename;

            if (!fileExists(full_path)) {
                // 연속된 번호가 끊기면 중단 (예: 0, 1 있고 2 없으면 종료)
                if (seq_idx > 0) ROS_INFO("Finished loading sequence files up to index %d.", seq_idx - 1);
                else ROS_WARN("No sequence files found in %s", base_dir_.c_str());
                break;
            }

            ROS_INFO("Found file: %s", filename.c_str());

            // 각 시퀀스마다 별도의 Publisher 생성
            // Topic 이름 예시: /lane_viz/seq_0, /lane_viz/seq_1
            std::string topic_name = "/lane_viz/seq_" + std::to_string(seq_idx);
            ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
            publishers_.push_back(pub); // 벡터에 저장해둠

            // 파일 로딩 및 발행 수행
            processFile(full_path, pub, seq_idx);

            seq_idx++;
        }
        
        server_.applyChanges(); // Interactive Marker 적용
    }

    void processFile(const std::string& path, ros::Publisher& pub, int seq_idx) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) return;

        uint32_t cluster_num = 0;
        ifs.read(reinterpret_cast<char*>(&cluster_num), 4);
        
        visualization_msgs::MarkerArray line_markers_msg;
        std::string seq_str = "seq_" + std::to_string(seq_idx); // "seq_0"

        for (uint32_t i = 0; i < cluster_num; ++i) {
            int32_t id, layer;
            uint32_t point_num;
            
            ifs.read(reinterpret_cast<char*>(&id), 4);
            ifs.read(reinterpret_cast<char*>(&layer), 4);
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

            // =========================================================
            // 1. Interactive Marker (클릭용)
            // =========================================================
            // seq가 다르면 id가 같아도 겹치지 않게 이름에 prefix 추가
            // 예: "seq_0/105", "seq_1/105"
            visualization_msgs::InteractiveMarker int_marker;
            int_marker.header.frame_id = "map";
            int_marker.name = seq_str + "/" + std::to_string(id); 
            int_marker.description = ""; // 텍스트 끄기 (너무 많음)

            visualization_msgs::InteractiveMarkerControl control;
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
            control.always_visible = true;

            visualization_msgs::Marker point_marker;
            point_marker.type = visualization_msgs::Marker::CUBE_LIST;
            // 시퀀스가 올라갈수록 점 크기를 조금씩 키우거나 모양을 다르게 해서 구별 가능하게 할 수도 있음
            point_marker.scale.x = 0.3; point_marker.scale.y = 0.3; point_marker.scale.z = 0.3;
            point_marker.color = color;
            
            for (const auto& p : lane_points) point_marker.points.push_back(p);
            if (!point_marker.points.empty()) control.markers.push_back(point_marker);
            
            int_marker.controls.push_back(control);
            server_.insert(int_marker); 

            // =========================================================
            // 2. Line Marker (시각화용)
            // =========================================================
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = ros::Time::now();
            
            // Namespace를 시퀀스별로 다르게 줄 수도 있지만, Topic이 다르므로 통일해도 됨.
            // 하지만 RViz 내에서 더 명확한 구분을 위해 ns에도 seq를 넣음
            line_marker.ns = "lines_" + seq_str; 
            line_marker.id = id;
            line_marker.type = visualization_msgs::Marker::LINE_LIST;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.15; // 선 두께
            line_marker.color = color;
            // 시퀀스 1(정렬됨)은 조금 더 밝거나 투명도를 다르게 주어 구별 가능 (선택사항)
            if (seq_idx == 1) { 
                line_marker.scale.x = 0.25; // 정렬된 건 좀 더 두껍게
                line_marker.color.a = 1.0; 
            } else {
                line_marker.color.a = 0.5; // 원본은 반투명
            }

            line_marker.pose.orientation.w = 1.0;

            for (size_t k = 0; k < lane_points.size(); ++k) {
                if (k + 1 < lane_points.size()) {
                    line_marker.points.push_back(lane_points[k]);
                    line_marker.points.push_back(lane_points[k+1]);
                }
            }

            if (!line_marker.points.empty()) {
                line_markers_msg.markers.push_back(line_marker);
            }
        }
        
        // 해당 시퀀스 토픽으로 발행
        pub.publish(line_markers_msg);
    }

    void spin() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    interactive_markers::InteractiveMarkerServer server_;
    std::vector<ros::Publisher> publishers_; // 여러 토픽을 관리하기 위한 벡터
    std::string base_dir_;
    double offset_x_, offset_y_, offset_z_;
    bool is_initialized_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bin_map_viewer");
    BinMapViewer viewer;
    viewer.spin(); 
    return 0;
}