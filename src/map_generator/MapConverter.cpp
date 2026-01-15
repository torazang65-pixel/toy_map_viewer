#include "toy_map_viewer/MapConverter.h"
#include "common/BinSaver.h"
#include "toy_map_viewer/LaneCleaner.h"
#include "toy_map_viewer/LaneMerger.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <limits>
#include <cmath>
#include <set>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

MapConverter::MapConverter() : nh_("~") {
    loadParameters();
}

void MapConverter::loadParameters() {
    // 파라미터 로딩
    nh_.param<std::string>("package_name", package_name_, "toy_map_viewer");
    nh_.param<int>("start_index", lane_config_.start_index, 20000);
    // nh_.param<int>("end_index", lane_config_.end_index, 21000);
    // nh_.param<int>("load_count", lane_config_.load_count, 1); // 필요 없는 파라미터
    nh_.param<std::string>("input_folder", input_folder_name_, "data/issue/global_maps/");
    nh_.param<std::string>("output_folder", output_folder_name_, "data/issue/output_bin/");
    nh_.param<bool>("crop_mode", lane_config_.crop_mode, true);
    // nh_.param<bool>("random_index", lane_config_.random_index, false);

    nh_.param<double>("overlap_radius", lane_config_.overlap_radius, 0.3);
    nh_.param<double>("linearity_tolerance", lane_config_.linearity_tolerance, 0.02);

    nh_.param<double>("search_radius", lane_config_.search_radius, 1.5);
    nh_.param<double>("direction_threshold_deg", lane_config_.direction_threshold_deg, 30.0);
    nh_.param<double>("weight_distance", lane_config_.weight_distance, 1.0);
    nh_.param<double>("weight_direction", lane_config_.weight_direction, 5.0);

    // 경로 설정
    std::string pkg_path = ros::package::getPath(package_name_);
    base_dir_ = pkg_path + "/" + input_folder_name_;
    output_dir_ = pkg_path + "/" + output_folder_name_ + std::to_string(lane_config_.start_index) + "/";

    // 출력 디렉토리 생성
    struct stat st = {0};
    if (stat(output_dir_.c_str(), &st) == -1) {
         std::string cmd = "mkdir -p " + output_dir_;
         system(cmd.c_str());
    }
}

std::map<int, Lane> MapConverter::filterDenseRegion(const std::map<int, Lane>& src_map, double crop_size_m) {
    if (src_map.empty()) return src_map;

    ROS_INFO(">>> Calculating density to find the busiest area...");

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& pair : src_map) {
        for (const auto& p : pair.second.points) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }
    }

    double grid_size = 50.0; 
    int cols = std::ceil((max_x - min_x) / grid_size);
    
    std::map<int, int> grid_counts; 
    int max_count = 0;
    int best_grid_idx = 0;

    for (const auto& pair : src_map) {
        for (const auto& p : pair.second.points) {
            int gx = (p.x - min_x) / grid_size;
            int gy = (p.y - min_y) / grid_size;
            int idx = gy * cols + gx;
            grid_counts[idx]++;
            if (grid_counts[idx] > max_count) {
                max_count = grid_counts[idx];
                best_grid_idx = idx;
            }
        }
    }

    int best_gy = best_grid_idx / cols;
    int best_gx = best_grid_idx % cols;
    double center_x = min_x + (best_gx * grid_size) + (grid_size / 2.0);
    double center_y = min_y + (best_gy * grid_size) + (grid_size / 2.0);

    ROS_INFO(">>> Most Dense Area Found! Center: (%.2f, %.2f), Count: %d", center_x, center_y, max_count);

    std::map<int, Lane> filtered_map;
    double half_size = crop_size_m / 2.0;

    for (const auto& pair : src_map) {
        Lane new_lane = pair.second;
        new_lane.points.clear();
        bool keep_lane = false;
        for (const auto& p : pair.second.points) {
            if (std::abs(p.x - center_x) <= half_size && std::abs(p.y - center_y) <= half_size) {
                new_lane.points.push_back(p);
                keep_lane = true;
            }
        }
        if (keep_lane && new_lane.points.size() > 1) {
            filtered_map[new_lane.id] = new_lane;
        }
    }
    return filtered_map;
}

void MapConverter::run() {
    ROS_INFO("========================================");
    ROS_INFO(">>> STARTING MAP CONVERTER");
    if (lane_config_.crop_mode) ROS_INFO(">>> CROP MODE IS: ON (True)");
    else ROS_INFO(">>> CROP MODE IS: OFF (False)");
    ROS_INFO("========================================");

    // 파일 로딩
    int current_idx = lane_config_.start_index;
    std::string filename = std::to_string(current_idx) + ".json";
    std::string file_path = base_dir_ + filename;
    std::ifstream f(file_path);

    ROS_INFO("Loading file: %s", file_path.c_str());

    if (f.is_open()){
        json data;
        try {
            f >> data;
            const auto& ids = data["roadgraph_samples/id"];
            const auto& xyz_flat = data["roadgraph_samples/xyz"];
            const auto& valids = data["roadgraph_samples/valid"];
            const auto& dir_flat = data["roadgraph_samples/dir"];
            const auto& type = data["roadgraph_samples/type"];
            const auto& explicit_vals = data["roadgraph_samples/explicit"];
            
            std::set<int> target_types = {6,7,8,9,10,11,12,13};

            for (size_t k = 0; k < ids.size(); ++k) {
                if (valids[k] != 1) continue;
                if (target_types.find((int)type[k]) == target_types.end()) continue;

                int id = ids[k];
                if (global_map_.find(id) == global_map_.end()) {
                    Lane new_lane; new_lane.id = id; 
                    new_lane.type = (int)type[k];
                    new_lane.explicit_lane = explicit_vals[k];
                    global_map_[id] = new_lane;
                }
                Point6D pt;
                pt.x = xyz_flat[3 * k]; pt.y = xyz_flat[3 * k + 1]; pt.z = xyz_flat[3 * k + 2];
                pt.dx = dir_flat[3 * k]; pt.dy = dir_flat[3 * k + 1]; pt.dz = dir_flat[3 * k + 2];
                global_map_[id].points.push_back(pt);
            }
            ROS_INFO("Original Total Lanes: %lu", global_map_.size());
        } catch (const std::exception& e) {
            ROS_ERROR("JSON parsing error: %s", e.what()); 
        }

        
    } else {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    // 처리 및 저장
    std::map<int, Lane>* map_to_save = &global_map_;
    std::map<int, Lane> cropped_map;

    if (lane_config_.crop_mode) {
        cropped_map = filterDenseRegion(global_map_, lane_config_.crop_size);
        map_to_save = &cropped_map;
        ROS_INFO("Filtered Lanes: %lu", map_to_save->size());
    }

    saveToBin(output_dir_ + "points_seq_0.bin", *map_to_save);

    ROS_INFO(">>> Reordering...");
    for (auto& pair : *map_to_save) LaneUtils::ReorderPoints(pair.second);
    saveToBin(output_dir_ + "points_seq_1.bin", *map_to_save);

    ROS_INFO(">>> Cleaning...");
    LaneCleaner::TrimOverlappingLaneEnds(*map_to_save, lane_config_);
    saveToBin(output_dir_ + "points_seq_2.bin", *map_to_save);

    ROS_INFO(">>> Merging...");
    LaneMerger::MergeFragmentedLanes(*map_to_save, lane_config_);
    saveToBin(output_dir_ + "points_seq_3.bin", *map_to_save);
    
    ROS_INFO(">>> MAP CONVERTER FINISHED.");
}