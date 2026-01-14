#pragma once

#include <vector>
#include <map>
#include <memory>
#include <opencv2/video/tracking.hpp>
#include <Eigen/Dense>
#include "common/DataTypes.h"

class LaneTracker {
public:
    struct Config {
        double match_distance_threshold = 2.5; // 매칭 허용 거리 (m) - 양방향이므로 조금 더 여유있게
        double match_angle_threshold = 20.0;   // 매칭 허용 각도 (도)
        double measurement_noise = 0.1;
        double process_noise = 0.01;
    };

    LaneTracker(const Config& config);
    ~LaneTracker() = default;

    std::map<int, Lane> process(const std::map<int, Lane>& detected_lanes);

private:
    struct Track {
        int id;
        Lane lane; // 누적된 차선 데이터
        
        // --- 양방향 확장을 위한 두 개의 KF ---
        
        // 1. Front Tracker (정방향 확장용)
        cv::KalmanFilter kf_front;
        cv::Mat state_front;
        Point6D predicted_front; // 다음 예상 위치 (앞쪽)

        // 2. Back Tracker (역방향 확장용)
        cv::KalmanFilter kf_back;
        cv::Mat state_back;
        Point6D predicted_back;  // 다음 예상 위치 (뒤쪽)
    };

    // KF 초기화 (Front는 끝점에서 시작, Back은 시작점에서 시작)
    void initializeTrack(Track& track, const Lane& segment);
    
    // 예측 및 업데이트
    Point6D predictKF(cv::KalmanFilter& kf);
    void updateKF(cv::KalmanFilter& kf, const Point6D& meas_pt, const Point6D& meas_dir);
    
    // 유틸리티
    double getAngleDiff(const Point6D& dir1, const Point6D& dir2);
    Point6D calculateBatchCenter(const std::vector<Lane>& lanes);

    Config config_;
    int track_id_counter_ = 0;
};