#pragma once

#include "AnimationLayer.h"
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>

class LaneLayer : public AnimationLayer {
public:
    LaneLayer(const std::string& name, ros::NodeHandle& nh, const std::string& topic, 
              const std::string& sub_folder, const std::string& file_prefix);
    
    ~LaneLayer() override = default;

    // 데이터 로드 (바이너리 파일 파싱 -> 마커 생성)
    void loadData(const std::string& base_dir, double off_x, double off_y, double off_z) override;
    
    // 특정 프레임 퍼블리시
    void publish(size_t frame_idx, const std_msgs::Header& header) override;
    
    // 데이터 비우기
    void clear() override;

protected:
    // 빈 화면 출력 (레이어 끌 때)
    void publishEmpty() override;

private:
    // 색상 생성 유틸리티
    std_msgs::ColorRGBA generateColor(int id);

private:
    ros::Publisher pub_;
    std::string sub_folder_;
    std::string file_prefix_;
    
    // 프레임별 마커 리스트 저장
    std::vector<visualization_msgs::MarkerArray> frames_;
};