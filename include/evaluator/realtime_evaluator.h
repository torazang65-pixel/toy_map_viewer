// include/realtime_line_generator/realtime_evaluator.h
#pragma once
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <common/data_types.h>
#include <common/io.h>
#include <linemapdraft_builder/polyline_evaluator.h> 
#include <vector>
#include <string>

namespace realtime_line_generator {

namespace ldb = linemapdraft_builder;
namespace lbe = linemapdraft_builder::polyline_evaluator;

struct VehiclePose {
    float x;           // 4 bytes
    float y;           // 4 bytes
    float z;           // 4 bytes
    float yaw;         // 4 bytes
    int32_t polyline_id; // 4 bytes
    uint32_t density;  // 4 bytes
};

class RealTimeEvaluator {
public:
    RealTimeEvaluator(ros::NodeHandle& nh);
    
    // 전체 GT 지도 및 차량 궤적 로드
    bool init(const std::string& gt_path, const std::string& traj_path);
    
    // 현재 프레임 평가 및 시각화 마커 발행
    void evaluateFrame(int frame_idx, const std::vector<std::vector<ldb::data_types::Point>>& draft_polylines, const std::string& frame_id);

    void saveEvaluationResults(int frame_idx, const std::string& fp_dir, const std::string& fn_dir);
    
    void printFinalSummary();
    
private:
    // 차량 위치 기준 ROI 필터링
    void filterPolylines(const std::vector<std::vector<ldb::data_types::Point>>& src,
                         std::vector<std::vector<ldb::data_types::Point>>& dst,
                         const VehiclePose& pose, float radius);

    // FP/FN 시각화를 위한 마커 생성 및 발행
    void publishMarkers(const std::vector<std::vector<ldb::data_types::Point>>& lines, 
                        ros::Publisher& pub, const std::string& ns, 
                        const std::string& frame_id, float r, float g, float b);

    
    std::vector<double> precisions_;
    std::vector<double> recalls_;
    std::vector<double> f1_scores_;
    
    std::vector<std::vector<ldb::data_types::Point>> full_gt_map_;
    std::vector<VehiclePose> vehicle_trajectory_;

    std::vector<std::vector<ldb::data_types::Point>> last_fps_;
    std::vector<std::vector<ldb::data_types::Point>> last_fns_;
    
    ros::Publisher fp_pub_; // False Positive (오탐) 퍼블리셔
    ros::Publisher fn_pub_; // False Negative (미탐) 퍼블리셔

    float eval_radius_;
    float dist_th_;
    float yaw_th_;
    float resample_dist_;
};

} // namespace realtime_line_generator