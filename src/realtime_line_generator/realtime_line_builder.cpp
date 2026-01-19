#include <realtime_line_generator/realtime_line_builder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <realtime_line_generator/polyline_generator.h> 
#include <realtime_line_generator/polyline_simplifier.h> 
#include <realtime_line_generator/post_processor.h> 
#include <chrono>

namespace realtime_line_generator {

namespace ldb = linemapdraft_builder;

RealTimeLineBuilder::RealTimeLineBuilder(ros::NodeHandle& nh) {
    nh.param("voxel_size", voxel_size_, 0.5f);
    nh.param("yaw_voxel_num", yaw_voxel_num_, 1);
    nh.param("min_density_threshold", min_density_th_, 1);
    nh.param("use_voxel_aging", use_voxel_aging_, true);
    nh.param("voxel_aging_interval", voxel_aging_interval_, 10);
    nh.param("max_voxel_age", max_voxel_age_, 100);

    voxel_manager_ = std::make_unique<RealTimeVoxelManager>(voxel_size_, yaw_voxel_num_);

    //polyline generator
    nh.param("neighbor_dist_thresh", neighbor_dist_thresh_, 2.0f);
    nh.param("cylinder_search_width", cylinder_search_width_, 1.0f);
    nh.param("alpha", alpha_, 0.7f);
    nh.param("beta", beta_, 0.9f);
    nh.param("drop_width", drop_width_, 1.0f);

    //polyline simplifier
    nh.param("use_vw", use_vw_, true);
    nh.param("use_rdp", use_rdp_, true);
    nh.param("vw_eps", vw_eps_, 0.5f);  // 면적 임계값
    nh.param("vw_eps_2", vw_eps_2_, 3.0f);  // 2차 단순화 임계값
    nh.param("rdp_eps", rdp_eps_, 0.5f); // 거리 임계값

    // Post Processor 파라미터 로드
    nh.param("merge_min_dist_th", merge_min_dist_th_, 1.0f);
    nh.param("merge_max_dist_th", merge_max_dist_th_, 10.0f);
    nh.param("merge_min_angle_th", merge_min_angle_th_, 0.1f);
    nh.param("merge_max_angle_th", merge_max_angle_th_, 0.3f);
    nh.param("min_polyline_length", min_polyline_length_, 3.0f);
}

void RealTimeLineBuilder::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::vector<ldb::data_types::Point> frame_points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), 
                                                    iter_y(*msg, "y"), 
                                                    iter_z(*msg, "z");

    sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
        ldb::data_types::Point p;
        p.x = *iter_x; 
        p.y = *iter_y; 
        p.z = *iter_z;
        p.yaw = *iter_i;
        p.density = static_cast<uint32_t>(*iter_i);
        p.polyline_id = ldb::data_types::Unclassified;
        frame_points.push_back(p);
    }

    // 1단계: 보셀화 진행
    processPipelineFrame(frame_points, msg->header.frame_id);
}

void RealTimeLineBuilder::processPipelineFrame(const std::vector<ldb::data_types::Point>& frame_points, const std::string& frame_id) {
    processPipeline(frame_points, frame_id);
    frame_counter_++;
}

void RealTimeLineBuilder::processPipeline(const std::vector<ldb::data_types::Point>& frame_points, const std::string& frame_id) {
    last_frame_points_ = frame_points;

    // 1. 새로운 점들을 보셀에 누적
    voxel_manager_->accumulate(frame_points, frame_counter_);

    // 2. 오래된 보셀 제거 (Aging) 
    // 매 프레임 호출하거나, CPU 부하를 줄이기 위해 n프레임마다 호출할 수 있습니다.
    if (use_voxel_aging_ && voxel_aging_interval_ > 0 &&
        (frame_counter_ % static_cast<uint32_t>(voxel_aging_interval_) == 0)) { 
        uint32_t max_age = static_cast<uint32_t>(max_voxel_age_);
        voxel_manager_->clearStaleVoxels(frame_counter_, max_age);
    }

    // 3. 필터링된 최신 보셀들만 가져와서 다음 단계 진행
    uint32_t min_density = min_density_th_ > 0 ? static_cast<uint32_t>(min_density_th_) : 0;
    auto active_voxels = voxel_manager_->getFilteredPoints(min_density);
    last_active_voxels_ = active_voxels;

    // 4. 폴리라인 생성/단순화 단계 호출
    processGeneratorStage(active_voxels, frame_id);

}

void RealTimeLineBuilder::processGeneratorStage(std::vector<ldb::data_types::Point>& active_voxels, const std::string& frame_id) {
    if (active_voxels.empty()) return;

    auto generator_start = std::chrono::steady_clock::now();

    // 1. 현재 활성화된 보셀들로 GridMap 빌드 (탐색 최적화)
    ldb::data_types::GridMap grid;
    float grid_size = neighbor_dist_thresh_ * 0.9f;
    ldb::data_types::build_grid_map(active_voxels, grid_size, grid);

    // 2. 폴리라인 생성 로직 실행
    std::vector<std::vector<ldb::data_types::Point>> polylines;
    
    // 실시간에서는 min_density를 0으로 두거나 파라미터화하여 조절
    ldb::polyline_generator::generate_polyline(
        active_voxels, polylines, grid, grid_size, 
        0, // min_density (이미 필터링된 보셀들이므로 0)
        neighbor_dist_thresh_, cylinder_search_width_, 
        alpha_, beta_, drop_width_
    );
    auto generator_end = std::chrono::steady_clock::now();
    timing_polyline_generator_ms_ +=
        std::chrono::duration<double, std::milli>(generator_end - generator_start).count();

    // 3. 폴리라인 단순화 적용
    auto simplifier_start = std::chrono::steady_clock::now();
    namespace lps = ldb::polyline_simplifier;
    for (auto& polyline : polylines) {
        if (polyline.size() <= 2) continue;

        if (use_vw_) {
            // 삼각형 면적 기반으로 중요하지 않은 점 제거
            lps::vw(polyline, vw_eps_);
        }
        if (use_rdp_) {
            // 직선과의 거리가 먼 점들만 남기는 RDP 적용
            lps::rdp(polyline, rdp_eps_);
        }
    }
    auto simplifier_end = std::chrono::steady_clock::now();
    timing_polyline_simplifier_ms_ +=
        std::chrono::duration<double, std::milli>(simplifier_end - simplifier_start).count();
    last_polylines_ = polylines;

    // 4. 포스트 프로세싱 (Post-Processor)
    namespace lpp = ldb::post_processor;
    auto merge_start = std::chrono::steady_clock::now();
    
    // 끊어진 차선 병합
    std::vector<std::vector<ldb::data_types::Point>> merged_polylines = lpp::mergeAllPolylines(
        polylines, merge_min_dist_th_, merge_max_dist_th_, 
        merge_min_angle_th_, merge_max_angle_th_
    );

    // 너무 짧은 차선 제거
    lpp::dropPolylines(merged_polylines, min_polyline_length_);

    // 2차 단순화
    for (auto& polyline : merged_polylines) {
        if (polyline.size() <= 2) {
            continue;
        }
        if (use_vw_) {
            lps::vw(polyline, vw_eps_2_);
        }
        if (use_rdp_) {
            lps::rdp(polyline, rdp_eps_);
        }
    }

    auto merge_end = std::chrono::steady_clock::now();
    timing_polyline_merge_ms_ +=
        std::chrono::duration<double, std::milli>(merge_end - merge_start).count();
    last_merged_polylines_ = merged_polylines;

    timing_frame_count_++;
}

const std::vector<ldb::data_types::Point>& RealTimeLineBuilder::getLastFramePoints() const {
    return last_frame_points_;
}

const std::vector<ldb::data_types::Point>& RealTimeLineBuilder::getLastActiveVoxels() const {
    return last_active_voxels_;
}

const std::vector<std::vector<ldb::data_types::Point>>& RealTimeLineBuilder::getLastPolylines() const {
    return last_polylines_;
}

void RealTimeLineBuilder::logTimingSummary() const {
    if (timing_frame_count_ == 0) {
        ROS_INFO("Timing summary: no frames processed.");
        return;
    }

    ROS_INFO(
        "Timing total (%zu frames): generator %.3f s, simplifier %.3f s, merge %.3f s",
        timing_frame_count_,
        timing_polyline_generator_ms_ / 1000.0,
        timing_polyline_simplifier_ms_ / 1000.0,
        timing_polyline_merge_ms_ / 1000.0);
}

const std::vector<std::vector<ldb::data_types::Point>>& RealTimeLineBuilder::getLastMergedPolylines() const {
    return last_merged_polylines_;
}

} // namespace realtime_line_generator
