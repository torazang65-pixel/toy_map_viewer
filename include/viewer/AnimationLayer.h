#pragma once

#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>

// 모든 애니메이션 레이어의 부모 클래스
class AnimationLayer {
public:
    AnimationLayer(const std::string& name, bool visible = true) 
        : name_(name), visible_(visible), frame_count_(0) {}
    
    virtual ~AnimationLayer() = default;

    // 데이터 로드 (오프셋 적용)
    virtual void loadData(const std::string& base_dir, double off_x, double off_y, double off_z) = 0;
    
    // 특정 프레임 Publish
    virtual void publish(size_t frame_idx, const std_msgs::Header& header) = 0;

    // 데이터 비우기
    virtual void clear() = 0;

    // 가시성 설정
    void setVisible(bool visible) { 
        visible_ = visible; 
        if (!visible_) publishEmpty(); // 꺼질 때 화면 지우기
    }
    
    bool isVisible() const { return visible_; }
    std::string getName() const { return name_; }
    size_t getFrameCount() const { return frame_count_; }

protected:
    // 가시성이 꺼졌을 때 화면을 지우는 함수 (구현체에서 정의)
    virtual void publishEmpty() = 0;

protected:
    std::string name_;
    bool visible_;
    size_t frame_count_;
};