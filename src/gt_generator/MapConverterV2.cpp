#include "gt_generator/MapConverterV2.h"
#include "common/io.h"

#include <filesystem>
#include <fstream>
#include <set>
#include <utility>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
#include <algorithm> // std::sort 사용
#include <cmath>     // std::hypot 사용
#include "gt_generator/LaneUtils.h"
#include "gt_generator/LaneCleaner.h"
#include "gt_generator/LaneMerger.h"

using json = nlohmann::json;
namespace ldb = linemapdraft_builder;

using linemapdraft_builder::data_types::Lane;
using linemapdraft_builder::data_types::Point6D;

MapConverterV2::MapConverterV2() : nh_("~") {
    loadParameters();
}

void MapConverterV2::loadParameters() {
    nh_.param<std::string>("package_name", package_name_, "realtime_line_generator");
    nh_.param<int>("file_idx", lane_config_.file_idx, 0);
    // Base input folder containing map_prev and map_latest directories
    nh_.param<std::string>("input_folder", base_input_dir_, "data/issue/");
    nh_.param<std::string>("output_folder", output_root_dir_, "data/issue/converted_bin/");

    std::string pkg_path = ros::package::getPath(package_name_);
    // Ensure trailing slash
    if (base_input_dir_.back() != '/') base_input_dir_ += "/";
    if (output_root_dir_.back() != '/') output_root_dir_ += "/";

    base_input_dir_ = pkg_path + "/" + base_input_dir_;
    output_root_dir_ = pkg_path + "/" + output_root_dir_ + std::to_string(lane_config_.file_idx) + "/";

    fs::create_directories(output_root_dir_);
}

void MapConverterV2::run() {
    // Process Previous Map (map_prev -> points_seq_0.bin)
    processMap("map_prev/", "points_seq_0.bin");
    
    // Process Latest Map (map_latest -> points_seq_1.bin)
    processMap("map_latest/", "points_seq_1.bin");
}

void MapConverterV2::processMap(const std::string& input_subdir, const std::string& output_filename) {
    global_map_.clear(); // Clear for fresh processing
    
    int current_idx = lane_config_.file_idx;
    std::string filename = std::to_string(current_idx) + ".json";
    std::string file_path = base_input_dir_ + input_subdir + filename;

    std::ifstream f(file_path);
    if (!f.is_open()) {
        ROS_WARN("[MapConverterV2] Failed to open file: %s", file_path.c_str());
        return;
    }

    json data;
    try {
        f >> data;
        const auto& ids = data["roadgraph_samples/id"];
        const auto& xyz_flat = data["roadgraph_samples/xyz"];
        const auto& valids = data["roadgraph_samples/valid"];
        const auto& dir_flat = data["roadgraph_samples/dir"];
        const auto& type = data["roadgraph_samples/type"];
        bool has_explicit = data.contains("roadgraph_samples/explicit");
        const json* explicit_vals = has_explicit ? &data["roadgraph_samples/explicit"] : nullptr;

        std::set<int> target_types = {6, 7, 8, 9, 10, 11, 12, 13};

        for (size_t k = 0; k < ids.size(); ++k) {
            if (valids[k] != 1) continue;
            if (target_types.find(static_cast<int>(type[k])) == target_types.end()) continue;

            int id = ids[k];
            if (global_map_.find(id) == global_map_.end()) {
                Lane new_lane;
                new_lane.id = id;
                new_lane.type = static_cast<int>(type[k]);
                bool explicit_lane = false;
                if (has_explicit && k < explicit_vals->size()) {
                    if ((*explicit_vals)[k].is_boolean()) {
                        explicit_lane = (*explicit_vals)[k].get<bool>();
                    } else if ((*explicit_vals)[k].is_number_integer()) {
                        explicit_lane = (*explicit_vals)[k].get<int>() != 0;
                    }
                }
                new_lane.explicit_lane = explicit_lane;
                global_map_[id] = new_lane;
            }

            Point6D pt;
            pt.x = xyz_flat[3 * k];
            pt.y = xyz_flat[3 * k + 1];
            pt.z = xyz_flat[3 * k + 2];
            pt.dx = dir_flat[3 * k];
            pt.dy = dir_flat[3 * k + 1];
            pt.dz = dir_flat[3 * k + 2];
            global_map_[id].points.push_back(pt);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("[MapConverterV2] JSON parsing error in %s: %s", file_path.c_str(), e.what());
        return;
    }

    std::vector<std::vector<ldb::data_types::Point>> polylines;
    polylines.reserve(global_map_.size());

    for (const auto& [id, lane] : global_map_) {
        std::vector<ldb::data_types::Point> polyline;
        polyline.reserve(lane.points.size());

        for (const auto& pt : lane.points) {
            ldb::data_types::Point p;
            p.x = static_cast<float>(pt.x);
            p.y = static_cast<float>(pt.y);
            p.z = static_cast<float>(pt.z);
            p.yaw = 0.0f;
            p.vz = 0.0f;
            p.polyline_id = id;
            p.density = 1;
            polyline.push_back(p);
        }
        polylines.push_back(std::move(polyline));
    }

    const std::string output_path = output_root_dir_ + output_filename;
    ldb::io::write_points_from_polylines(output_path, polylines);
    ROS_INFO("[MapConverterV2] Saved %s (%lu lanes).", output_filename.c_str(), polylines.size());
    ROS_INFO("Saved points_seq_0.bin (%lu lanes).", polylines.size());

    // 3. 점 순서 재정렬 (Greedy Reordering)
    // "선이 뭉치는 현상"을 해결하는 핵심 로직입니다. 
    // 각 차선 내부의 점들을 물리적인 주행 순서대로 다시 배치합니다.
    for (auto& [id, lane] : global_map_) LaneUtils::ReorderPoints(lane); 

    // 1. 중복 및 겹치는 끝단 제거
    // 근접한 차선끼리 겹치는 점들을 제거하여 데이터 밀도를 균일하게 만듭니다.
    LaneCleaner::TrimOverlappingLaneEnds(global_map_, lane_config_);

    // 2. 파편화된 차선 병합
    // 끊어져 있는 차선들을 거리와 각도 기반으로 판단하여 하나의 ID로 합칩니다.
    // 이를 통해 Recall 수치가 비약적으로 상승합니다.
    LaneMerger::MergeFragmentedLanes(global_map_, lane_config_);

    // ---------------------------------------------------------
    // [STEP 4] 최종 정제된 데이터를 gt.bin으로 저장
    // ---------------------------------------------------------
    std::vector<std::vector<ldb::data_types::Point>> gt_polylines;
    
    for (const auto& [id, lane] : global_map_) {
        if (lane.points.size() < 2) continue;

        std::vector<ldb::data_types::Point> segment;
        for (const auto& pt : lane.points) {
            ldb::data_types::Point p;
            p.x = static_cast<float>(pt.x);
            p.y = static_cast<float>(pt.y);
            p.z = static_cast<float>(pt.z);
            p.polyline_id = id;
            p.density = 1;
            segment.push_back(p);
        }
        gt_polylines.push_back(std::move(segment));
    }

    // 정렬과 병합이 완료된 깨끗한 데이터를 구조화된 폴리라인 형식으로 저장합니다.
    ldb::io::write_polylines(output_root_dir_ + "gt.bin", gt_polylines);
    ROS_INFO("Saved refined gt.bin (%lu merged lanes).", gt_polylines.size());
}
