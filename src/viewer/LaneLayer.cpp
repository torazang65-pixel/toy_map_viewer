#include "viewer/LaneLayer.h"
#include <fstream>
#include <sys/stat.h>
#include <cmath>
#include <array>

// 파일 존재 여부 확인 헬퍼
namespace {
    bool fileExists(const std::string& name) {
        struct stat buffer; return (stat (name.c_str(), &buffer) == 0); 
    }
}

LaneLayer::LaneLayer(const std::string& name, ros::NodeHandle& nh, const std::string& topic, 
                     const std::string& sub_folder, const std::string& file_prefix)
    : AnimationLayer(name), sub_folder_(sub_folder), file_prefix_(file_prefix) {
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

std_msgs::ColorRGBA LaneLayer::generateColor(int id) {
    // ID 기반 랜덤 색상 생성 (HSV -> RGB 변환 간소화)
    uint32_t hash = id * 2654435761; 
    float h = (hash % 360) / 360.0f;
    float s = 0.8f; 
    float v = 1.0f;

    auto hsv2rgb = [](float h, float s, float v) -> std::array<float, 3> {
        int i = int(h * 6);
        float f = h * 6 - i;
        float p = v * (1 - s);
        float q = v * (1 - f * s);
        float t = v * (1 - (1 - f) * s);
        switch(i % 6) {
            case 0: return {v, t, p};
            case 1: return {q, v, p};
            case 2: return {p, v, t};
            case 3: return {p, q, v};
            case 4: return {t, p, v};
            case 5: return {v, p, q};
            default: return {0,0,0};
        }
    };

    auto rgb = hsv2rgb(h, s, v);
    std_msgs::ColorRGBA color;
    color.r = rgb[0]; color.g = rgb[1]; color.b = rgb[2]; color.a = 1.0f;
    return color;
}

void LaneLayer::loadData(const std::string& base_dir, double off_x, double off_y, double off_z) {
    clear();
    int idx = 0;
    
    ROS_INFO("[%s] Loading data from %s...", name_.c_str(), sub_folder_.c_str());

    while(true) {
        // 파일 경로: .../lanes/lane_0.bin
        std::string path = base_dir + sub_folder_ + "/" + file_prefix_ + "_" + std::to_string(idx) + ".bin";
        if (!fileExists(path)) break;

        visualization_msgs::MarkerArray marker_array;
        std::ifstream ifs(path, std::ios::binary);
        
        if (ifs.is_open()) {
            uint32_t cluster_num = 0;
            ifs.read(reinterpret_cast<char*>(&cluster_num), 4);
            
            for (uint32_t c = 0; c < cluster_num; ++c) {
                int32_t id, layer; 
                uint32_t p_num;
                bool explicit_lane;

                // 헤더 파싱 (BinSaver.h의 saveToBin 구조 참조)
                ifs.read(reinterpret_cast<char*>(&id), 4);
                ifs.read(reinterpret_cast<char*>(&layer), 4);
                ifs.read(reinterpret_cast<char*>(&explicit_lane), sizeof(bool));
                ifs.read(reinterpret_cast<char*>(&p_num), 4);

                visualization_msgs::Marker marker;
                marker.header.frame_id = "map"; // map 좌표계 기준
                marker.ns = "generated_lanes";
                marker.id = id + (idx * 10000); // 프레임별 ID 충돌 방지용 오프셋
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.2; // 선 두께
                marker.color = generateColor(id);

                for (uint32_t j = 0; j < p_num; ++j) {
                    float x, y, z;
                    ifs.read(reinterpret_cast<char*>(&x), 4);
                    ifs.read(reinterpret_cast<char*>(&y), 4);
                    ifs.read(reinterpret_cast<char*>(&z), 4);

                    geometry_msgs::Point p;
                    // 글로벌 좌표계 오프셋 적용
                    p.x = x - off_x; 
                    p.y = y - off_y; 
                    p.z = z - off_z;
                    marker.points.push_back(p);
                }
                marker_array.markers.push_back(marker);
            }
            ifs.close();
        }
        frames_.push_back(marker_array);
        idx++;
    }
    frame_count_ = frames_.size();
    ROS_INFO("[%s] Loaded %lu frames.", name_.c_str(), frame_count_);
}

void LaneLayer::publish(size_t frame_idx, const std_msgs::Header& header) {
    if (!visible_ || frames_.empty()) return;
    
    size_t idx = frame_idx % frames_.size();
    
    visualization_msgs::MarkerArray& msg = frames_[idx];
    
    // 헤더 타임스탬프 동기화
    for(auto& marker : msg.markers) {
        marker.header.stamp = header.stamp;
    }
    pub_.publish(msg);
}

void LaneLayer::publishEmpty() {
    visualization_msgs::MarkerArray empty;
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    empty.markers.push_back(delete_all);
    pub_.publish(empty);
}

void LaneLayer::clear() {
    frames_.clear();
    frame_count_ = 0;
}