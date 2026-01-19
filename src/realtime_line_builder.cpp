#include <toy_map_viewer/realtime_line_builder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <toy_map_viewer/polyline_generator.h> 
#include <toy_map_viewer/polyline_simplifier.h> 
#include <toy_map_viewer/post_processor.h> 
#include <visualization_msgs/MarkerArray.h>

namespace toy_map_viewer {

namespace ldb = linemapdraft_builder;

RealTimeLineBuilder::RealTimeLineBuilder(ros::NodeHandle& nh) {
    nh.param("voxel_size", voxel_size_, 0.5f);
    nh.param("yaw_voxel_num", yaw_voxel_num_, 1);
    nh.param("max_voxel_age", max_voxel_age_, 100);

    voxel_manager_ = std::make_unique<RealTimeVoxelManager>(voxel_size_, yaw_voxel_num_);
    voxel_pub_ = nh.advertise<sensor_msgs::PointCloud2>("active_voxels", 1);

    //polyline generator
    nh.param("neighbor_dist_thresh", neighbor_dist_thresh_, 2.0f);
    nh.param("cylinder_search_width", cylinder_search_width_, 1.0f);
    nh.param("alpha", alpha_, 0.7f);
    nh.param("beta", beta_, 0.9f);
    nh.param("drop_width", drop_width_, 1.0f);

    polyline_pub_ = nh.advertise<visualization_msgs::MarkerArray>("detected_polylines", 1);

    // polyline simplifier
    nh.param("VW_eps_1", vw_eps_1_, 0.5f);

    // Post Processor 파라미터 로드
    nh.param("merge_min_dist_th", merge_min_dist_th_, 1.0f);
    nh.param("merge_max_dist_th", merge_max_dist_th_, 10.0f);
    nh.param("merge_min_angle_th", merge_min_angle_th_, 0.1f);
    nh.param("merge_max_angle_th", merge_max_angle_th_, 0.3f);
    nh.param("min_polyline_length", min_polyline_length_, 3.0f);

    // polyline simplifier 2
    nh.param("VW_eps_2", vw_eps_2_, 3.0f);
    nh.param("RDP_eps", rdp_eps_, 0.5f);
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
        p.polyline_id = ldb::data_types::Unclassified;
        frame_points.push_back(p);
    }

    // 1단계: 보셀화 진행
    processPipeline(frame_points, msg->header.frame_id);
    frame_counter_++;
}

void RealTimeLineBuilder::processPipeline(const std::vector<ldb::data_types::Point>& frame_points, const std::string& frame_id) {
    // 1. 새로운 점들을 보셀에 누적
    voxel_manager_->accumulate(frame_points, frame_counter_);

    // 2. 오래된 보셀 제거 (Aging) 
    // 매 프레임 호출하거나, CPU 부하를 줄이기 위해 n프레임마다 호출할 수 있습니다.
    if (frame_counter_ % 10 == 0) { 
        voxel_manager_->clearStaleVoxels(frame_counter_, max_voxel_age_);
    }

    // 3. 필터링된 최신 보셀들만 가져와서 다음 단계 진행
    auto active_voxels = voxel_manager_->getFilteredPoints();
    publishVoxelCloud(active_voxels, frame_id);

    // 4. 폴리라인 생성/단순화 단계 호출
    processGeneratorStage(active_voxels, frame_id);

}

void RealTimeLineBuilder::publishVoxelCloud(const std::vector<linemapdraft_builder::data_types::Point>& voxels, const std::string& frame_id) {
    if (voxels.empty()) return;

    sensor_msgs::PointCloud2 output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = frame_id;

    // 1. PointCloud2 필드 구성 변경: "xyz" + "intensity"
    sensor_msgs::PointCloud2Modifier modifier(output_msg);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(voxels.size());

    // 2. 각 필드에 대한 Iterator 생성
    sensor_msgs::PointCloud2Iterator<float> out_x(output_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_i(output_msg, "intensity"); // Intensity 반복자 추가

    // 3. 데이터 할당 루프
    for (const auto& v : voxels) {
        *out_x = v.x;
        *out_y = v.y;
        *out_z = v.z;
        *out_i = v.yaw; // yaw 각도 값을 intensity 필드에 저장

        ++out_x; ++out_y; ++out_z; ++out_i;
    }

    voxel_pub_.publish(output_msg);
}

void RealTimeLineBuilder::processGeneratorStage(std::vector<ldb::data_types::Point>& active_voxels, const std::string& frame_id) {
    if (active_voxels.empty()) return;

    // -------------------------------------------------------
    // [STEP 1] Polyline 생성
    // -------------------------------------------------------
    ldb::data_types::GridMap grid;
    float grid_size = neighbor_dist_thresh_ * 0.9f;
    ldb::data_types::build_grid_map(active_voxels, grid_size, grid);

    // 폴리라인 생성 로직 실행
    std::vector<std::vector<ldb::data_types::Point>> polylines;
    
    // 실시간에서는 min_density를 0으로 두거나 파라미터화하여 조절
    ldb::polyline_generator::generate_polyline(
        active_voxels, polylines, grid, grid_size, 
        0, // min_density (이미 필터링된 보셀들이므로 0)
        neighbor_dist_thresh_, cylinder_search_width_, 
        alpha_, beta_, drop_width_
    );

    // -------------------------------------------------------
    // [STEP 2] 1차 단순화 (VW만 적용, 작은 노이즈 제거)
    // -------------------------------------------------------
    namespace lps = ldb::polyline_simplifier;
    for (auto& polyline : polylines) {
        if (polyline.size() > 2) {
            lps::vw(polyline, vw_eps_1_); 
        }
    }

    // -------------------------------------------------------
    // [STEP 3] 포스트 프로세싱 (병합 및 짧은 선 제거)
    // -------------------------------------------------------
    namespace lpp = ldb::post_processor;
    
    // 끊어진 차선 병합
    std::vector<std::vector<ldb::data_types::Point>> merged_polylines = lpp::mergeAllPolylines(
        polylines, merge_min_dist_th_, merge_max_dist_th_, 
        merge_min_angle_th_, merge_max_angle_th_
    );

    // 너무 짧은 차선 제거
    lpp::dropPolylines(merged_polylines, min_polyline_length_);

    // -------------------------------------------------------
    // [STEP 4] 2차 단순화 (병합 후 최종 정밀화: VW + RDP)
    // -------------------------------------------------------
    // for (auto& polyline : merged_polylines) {
    //     if (polyline.size() > 2) {
    //         lps::vw(polyline, vw_eps_2_); // 더 큰 면적 임계값 적용
    //         lps::rdp(polyline, rdp_eps_); // RDP로 직선 구간 단순화
    //     }
    // }

    // -------------------------------------------------------
    // [STEP 5] 시각화 발행
    // -------------------------------------------------------
    publishPolylines(merged_polylines, frame_id);
}

void RealTimeLineBuilder::publishPolylines(const std::vector<std::vector<ldb::data_types::Point>>& polylines, const std::string& frame_id) {
    visualization_msgs::MarkerArray marker_array;
    
    // 이전 마커 삭제 (Delete All)
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < polylines.size(); ++i) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = frame_id;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "polylines";
        line_strip.id = i;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        
        line_strip.scale.x = 0.2; // 선 두께
        line_strip.color.r = 0.0; line_strip.color.g = 1.0; line_strip.color.b = 0.0; line_strip.color.a = 1.0;

        for (const auto& pt : polylines[i]) {
            geometry_msgs::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            line_strip.points.push_back(p);
        }
        marker_array.markers.push_back(line_strip);
    }
    polyline_pub_.publish(marker_array);
}

} // namespace toy_map_viewer


