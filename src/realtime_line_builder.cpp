#include <toy_map_viewer/realtime_line_builder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <toy_map_viewer/polyline_generator.h> 
#include <visualization_msgs/MarkerArray.h>

namespace toy_map_viewer {

namespace ldb = linemapdraft_builder;

RealTimeLineBuilder::RealTimeLineBuilder(ros::NodeHandle& nh) {
    nh.param("voxel_size", voxel_size_, 0.5f);
    nh.param("yaw_voxel_num", yaw_voxel_num_, 1);

    voxel_manager_ = std::make_unique<RealTimeVoxelManager>(voxel_size_, yaw_voxel_num_);
    voxel_pub_ = nh.advertise<sensor_msgs::PointCloud2>("active_voxels", 1);

    nh.param("neighbor_dist_thresh", neighbor_dist_thresh_, 2.0f);
    nh.param("cylinder_search_width", cylinder_search_width_, 1.0f);
    nh.param("alpha", alpha_, 0.7f);
    nh.param("beta", beta_, 0.9f);
    nh.param("drop_width", drop_width_, 1.0f);

    polyline_pub_ = nh.advertise<visualization_msgs::MarkerArray>("detected_polylines", 1);

    nh.param("use_vw", use_vw_, true);
    nh.param("use_rdp", use_rdp_, true);
    nh.param("vw_eps", vw_eps_, 0.5f);  // 면적 임계값
    nh.param("rdp_eps", rdp_eps_, 0.1f); // 거리 임계값
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
        uint32_t max_age = 100; // 예: 100프레임(약 10초) 동안 업데이트 없으면 삭제
        voxel_manager_->clearStaleVoxels(frame_counter_, max_age);
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

    // 3. [추가] 폴리라인 단순화 적용
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

    // 3. 시각화 (MarkerArray 발행)
    publishPolylines(polylines, frame_id);
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


