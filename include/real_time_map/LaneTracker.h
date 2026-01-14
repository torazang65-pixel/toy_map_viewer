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
        double match_distance_threshold = 1.5; // 매칭 허용 거리 (m)
        double match_angle_threshold = 20.0;   // 매칭 허용 각도 (도)
        double measurement_noise = 0.1;        // R 값 (작을수록 측정값 신뢰)
        double process_noise = 0.01;           // Q 값 (작을수록 예측값/관성 신뢰)
    };

    LaneTracker(const Config& config);
    ~LaneTracker() = default;

    // RANSAC에서 나온 Lane들을 입력받아 KF로 병합된 Lane들을 반환
    std::map<int, Lane> process(const std::map<int, Lane>& detected_lanes);

private:
    struct Track {
        int id;
        Lane lane; // 현재까지 누적된 Lane 데이터
        
        cv::KalmanFilter kf;
        cv::Mat state; // [x, y, z, dx, dy, dz]
        cv::Mat meas;  // [x, y, z, dx, dy, dz]
        
        Point6D predicted_head; // KF가 예측한 다음 위치
    };

    // 내부 유틸리티 함수
    void initializeKF(Track& track, const Point6D& start_pt, const Point6D& dir);
    Point6D predictKF(Track& track);
    void updateKF(Track& track, const Point6D& measurement_pt, const Point6D& measurement_dir);
    
    // 두 벡터 사이의 각도 차이 계산 (도 단위)
    double getAngleDiff(const Point6D& dir1, const Point6D& dir2);
    
    // 배치의 중심점 계산 (Center-Out 전략용)
    Point6D calculateBatchCenter(const std::vector<Lane>& lanes);

    Config config_;
    int track_id_counter_ = 0;
};