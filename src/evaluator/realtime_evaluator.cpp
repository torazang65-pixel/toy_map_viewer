// src/realtime_evaluator.cpp
#include <evaluator/realtime_evaluator.h>
#include <evaluator/polyline_evaluator.h>
#include <common/geometry_utils.h>
#include <numeric>

namespace realtime_line_generator {

namespace lbe = linemapdraft_builder::polyline_evaluator;

RealTimeEvaluator::RealTimeEvaluator(ros::NodeHandle& nh) {
    nh.param("eval_radius", eval_radius_, 30.0f); // default: 반경 100m
    nh.param("dist_th", dist_th_, 0.5f);
    nh.param("yaw_th", yaw_th_, 0.8f);
    nh.param("resample_distance", resample_dist_, 0.3f);

    fp_pub_ = nh.advertise<visualization_msgs::MarkerArray>("eval/false_positives", 1);
    fn_pub_ = nh.advertise<visualization_msgs::MarkerArray>("eval/false_negatives", 1);
}

bool RealTimeEvaluator::init(const std::string& gt_path, const std::string& traj_path) {
    // 1. 전체 GT 로드 
    if (!ldb::io::load_polylines(gt_path, full_gt_map_)) return false;

    // 2. 차량 궤적 로드 
    std::ifstream ifs(traj_path, std::ios::binary);
    if (!ifs) return false;

    // 파일 처음에 기록된 4바이트 헤더(포즈 개수)를 먼저 읽습니다.
    uint32_t num_poses = 0;
    ifs.read(reinterpret_cast<char*>(&num_poses), 4);

    if (num_poses > 1000000) { // 비정상적인 값 방지
        ROS_ERROR("Invalid trajectory size: %u", num_poses);
        return false;
    }

    // 개수만큼 정확히 읽어옵니다.
    vehicle_trajectory_.clear();
    vehicle_trajectory_.resize(num_poses);
    for (uint32_t i = 0; i < num_poses; ++i) {
        ifs.read(reinterpret_cast<char*>(&vehicle_trajectory_[i]), sizeof(VehiclePose));
    }

    ifs.close();
    ROS_INFO("Loaded %zu vehicle poses from %s", vehicle_trajectory_.size(), traj_path.c_str());
    return true;
}

void RealTimeEvaluator::evaluateFrame(int frame_idx, const std::vector<std::vector<ldb::data_types::Point>>& draft_polylines, const std::string& frame_id) {
    if (frame_idx < 0 || frame_idx >= vehicle_trajectory_.size()) return;
    
    const VehiclePose& current_pose = vehicle_trajectory_[frame_idx];

    ROS_INFO_STREAM_ONCE("Check Pose: x=" << current_pose.x << ", y=" << current_pose.y);
    if (!full_gt_map_.empty() && !full_gt_map_[0].empty()) {
        ROS_INFO_STREAM_ONCE("Check GT: x=" << full_gt_map_[0][0].x << ", y=" << full_gt_map_[0][0].y);
    }

    // [Step 1] ROI 필터링 (반경 eval_radius_)
    std::vector<std::vector<ldb::data_types::Point>> roi_gt, roi_draft;
    filterPolylines(full_gt_map_, roi_gt, current_pose, eval_radius_);
    filterPolylines(draft_polylines, roi_draft, current_pose, eval_radius_);

    if (roi_gt.empty() && roi_draft.empty()) return;

    // [Step 2] 리샘플링
    std::vector<std::vector<ldb::data_types::Point>> gt_resampled, draft_resampled;
    lbe::resample(roi_gt, gt_resampled, resample_dist_);
    lbe::resample(roi_draft, draft_resampled, resample_dist_);

    // [Step 3] Length Coverage 평가
    std::vector<std::vector<ldb::data_types::Point>> inter_d, inter_g, fps, fns;
    lbe::evaluate_len_coverage(draft_resampled, gt_resampled, inter_d, inter_g, fps, fns, dist_th_, yaw_th_);

    // 프레임별 지표 계산 및 저장
    double tp_p = lbe::total_length(inter_d); // Draft 기준 매칭된 길이
    double tp_r = lbe::total_length(inter_g); // GT 기준 매칭된 길이
    double fp_l = lbe::total_length(fps);     // 오탐 길이
    double fn_l = lbe::total_length(fns);     // 미탐 길이

    double p = (tp_p + fp_l > 0) ? tp_p / (tp_p + fp_l) : 0.0;
    double r = (tp_r + fn_l > 0) ? tp_r / (tp_r + fn_l) : 0.0;
    double f1 = (p + r > 0) ? (2 * p * r) / (p + r) : 0.0;

    precisions_.push_back(p);
    recalls_.push_back(r);
    f1_scores_.push_back(f1);
    
    // [Step 4] FP/FN 마커 발행
    publishMarkers(fps, fp_pub_, "false_positives", frame_id, 1.0, 0.0, 0.0); // Red: 오탐
    publishMarkers(fns, fn_pub_, "false_negatives", frame_id, 0.0, 0.0, 1.0); // Blue: 미탐

    last_fps_ = fps; 
    last_fns_ = fns;
}

void RealTimeEvaluator::printFinalSummary() {
    if (precisions_.empty()) {
        ROS_WARN("[Evaluator] No frames evaluated.");
        return;
    }

    double sum_p = std::accumulate(precisions_.begin(), precisions_.end(), 0.0);
    double sum_r = std::accumulate(recalls_.begin(), recalls_.end(), 0.0);
    double sum_f1 = std::accumulate(f1_scores_.begin(), f1_scores_.end(), 0.0);
    size_t n = precisions_.size();

    std::cout << "\n========================================" << std::endl;
    std::cout << "   FINAL REAL-TIME EVALUATION SUMMARY   " << std::endl;
    std::cout << "   (Averaged over " << n << " frames)   " << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << " Average Precision: " << sum_p / n << std::endl;
    std::cout << " Average Recall:    " << sum_r / n << std::endl;
    std::cout << " Average F1-score:  " << sum_f1 / n << std::endl;
    std::cout << "========================================\n" << std::endl;
}

// realtime_evaluator.cpp 내에 추가
void RealTimeEvaluator::saveEvaluationResults(int frame_idx, const std::string& fp_dir, const std::string& fn_dir) {
    // 마지막 evaluateFrame에서 계산된 fps, fns를 사용 (멤버 변수로 유지 필요)
    std::string fp_path = fp_dir + "/fp_" + std::to_string(frame_idx) + ".bin";
    std::string fn_path = fn_dir + "/fn_" + std::to_string(frame_idx) + ".bin";

    ldb::io::write_polylines(fp_path, last_fps_); // last_fps_는 evaluateFrame에서 저장된 결과
    ldb::io::write_polylines(fn_path, last_fns_);
}

void RealTimeEvaluator::filterPolylines(const std::vector<std::vector<ldb::data_types::Point>>& src,
                                       std::vector<std::vector<ldb::data_types::Point>>& dst,
                                       const VehiclePose& pose, float radius) {
    for (const auto& polyline : src) {
        std::vector<ldb::data_types::Point> filtered;
        for (const auto& pt : polyline) {
            float d = std::hypot(pt.x - pose.x, pt.y - pose.y);
            if (d < radius) filtered.push_back(pt);
        }
        if (filtered.size() >= 2) dst.push_back(filtered);
    }
}

void RealTimeEvaluator::publishMarkers(const std::vector<std::vector<ldb::data_types::Point>>& lines, 
                                      ros::Publisher& pub, const std::string& ns, 
                                      const std::string& frame_id, float r, float g, float b) {
    visualization_msgs::MarkerArray msg;
    
    // 이전 마커 삭제
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(del);

    for (size_t i = 0; i < lines.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.15; // 선 두께
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 0.8;

        for (const auto& pt : lines[i]) {
            geometry_msgs::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            marker.points.push_back(p);
        }
        msg.markers.push_back(marker);
    }
    pub.publish(msg);
}

} // namespace realtime_line_generator