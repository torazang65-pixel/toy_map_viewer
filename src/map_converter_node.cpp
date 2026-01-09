#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <sys/stat.h>
#include <limits> // min, max 사용
#include <cmath>  // abs 사용
#include <set>
#include <nlohmann/json.hpp> 

#include "toy_map_viewer/DataTypes.h"
#include "toy_map_viewer/BinSaver.h"
#include "toy_map_viewer/LaneUtils.h"
#include "toy_map_viewer/LaneCleaner.h"
#include "toy_map_viewer/LaneMerger.h"

using json = nlohmann::json;

std::map<int, Lane> global_map;

// =========================================================
// [핵심 기능] 가장 밀도가 높은 구역을 찾아 자르는 함수
// =========================================================
std::map<int, Lane> filterDenseRegion(const std::map<int, Lane>& src_map, double crop_size_m = 200.0) {
    if (src_map.empty()) return src_map;

    ROS_INFO(">>> Calculating density to find the busiest area...");

    // 1. 전체 맵의 범위(Min/Max) 계산
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

    // 2. 격자(Grid) 만들기 (50m 단위로 쪼개기)
    double grid_size = 50.0; 
    int cols = std::ceil((max_x - min_x) / grid_size);
    int rows = std::ceil((max_y - min_y) / grid_size);
    
    // 격자별 점 개수 카운팅 (Key: index, Value: count)
    std::map<int, int> grid_counts; 
    int max_count = 0;
    int best_grid_idx = 0;

    for (const auto& pair : src_map) {
        for (const auto& p : pair.second.points) {
            int gx = (p.x - min_x) / grid_size;
            int gy = (p.y - min_y) / grid_size;
            int idx = gy * cols + gx; // 1차원 인덱스로 변환

            grid_counts[idx]++;
            
            if (grid_counts[idx] > max_count) {
                max_count = grid_counts[idx];
                best_grid_idx = idx;
            }
        }
    }

    // 3. 가장 점이 많은 격자의 중심 좌표 계산
    int best_gy = best_grid_idx / cols;
    int best_gx = best_grid_idx % cols;
    double center_x = min_x + (best_gx * grid_size) + (grid_size / 2.0);
    double center_y = min_y + (best_gy * grid_size) + (grid_size / 2.0);

    ROS_INFO(">>> Most Dense Area Found!");
    ROS_INFO("    Center: (%.2f, %.2f)", center_x, center_y);
    ROS_INFO("    Max Points in 50m grid: %d", max_count);
    ROS_INFO(">>> Cropping map (Size: %.0fm x %.0fm)...", crop_size_m, crop_size_m);

    // 4. 중심 기준 crop_size 안쪽의 데이터만 필터링
    std::map<int, Lane> filtered_map;
    double half_size = crop_size_m / 2.0;

    for (const auto& pair : src_map) {
        Lane new_lane = pair.second;
        new_lane.points.clear(); // 점 비우고 다시 채움

        bool keep_lane = false;
        for (const auto& p : pair.second.points) {
            // 범위 안에 들어오는지 체크
            if (std::abs(p.x - center_x) <= half_size && 
                std::abs(p.y - center_y) <= half_size) {
                new_lane.points.push_back(p);
                keep_lane = true;
            }
        }
        
        // 유효한 점이 하나라도 있는 차선만 저장
        if (keep_lane && new_lane.points.size() > 1) {
            filtered_map[new_lane.id] = new_lane;
        }
    }

    return filtered_map;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "map_converter_node");
    ros::NodeHandle nh("~");

    std::string package_name;
    std::string file_name; // 추가
    std::string input_folder_name;
    std::string output_folder_name;
    LaneConfig lane_config;

    nh.param<std::string>("package_name", package_name, "toy_map_viewer");
    nh.param<std::string>("file_name", file_name, "20032"); // 추가
    nh.param<int>("load_count", lane_config.load_count, 1);
    nh.param<std::string>("input_folder", input_folder_name, "data/issue/global_maps/");
    nh.param<std::string>("output_folder", output_folder_name, "data/issue/output_bin/");
    nh.param<bool>("crop_mode", lane_config.crop_mode, true);

    // [Lane Cleaner Config]
    nh.param<double>("overlap_radius", lane_config.overlap_radius, 0.3);
    nh.param<double>("linearity_tolerance", lane_config.linearity_tolerance, 0.02);

    // [Lane Merger Config]
    nh.param<double>("search_radius", lane_config.search_radius, 1.5);
    nh.param<double>("direction_threshold_deg", lane_config.direction_threshold_deg, 30.0);
    nh.param<double>("weight_distance", lane_config.weight_distance, 1.0);
    nh.param<double>("weight_direction", lane_config.weight_direction, 5.0);
    ROS_INFO("========================================");
    if (lane_config.crop_mode) {
        ROS_INFO(">>> CROP MODE IS: ON (True)");
    } else {
        ROS_INFO(">>> CROP MODE IS: OFF (False) -> Saving FULL MAP");
    }
    ROS_INFO("========================================");

    bool random_index = lane_config.random_index;
    int start_index;
    int range = lane_config.end_index - lane_config.start_index;
    if(random_index){
        if(range >0 ){
            ROS_INFO("Random Index Mode ON: Selecting start index between %d and %d", lane_config.start_index, lane_config.end_index -1);
            srand(time(0));
            start_index = lane_config.start_index + (rand() % range);
        } else {
            ROS_WARN("Invalid range for random index selection. Check start_index and end_index parameters.");
            start_index = lane_config.start_index;
        }
    } else {
        start_index = lane_config.start_index;
    }
    int load_count = lane_config.load_count;
    bool crop_mode = lane_config.crop_mode;

    std::string pkg_path = ros::package::getPath(package_name);
    std::string base_dir = pkg_path + "/" + input_folder_name;
    std::string output_dir = pkg_path + "/" + output_folder_name + file_name + "/";

    struct stat st = {0};
    if (stat(output_dir.c_str(), &st) == -1) {
         std::string cmd = "mkdir -p " + output_dir;
         system(cmd.c_str());
    }

    ROS_INFO("Loading file: %s.json from %s", file_name.c_str(), base_dir.c_str());

    // --- 파일 로딩 Loop (기존과 동일) ---
    for (int i = 0; i < lane_config.load_count; ++i) {
        // file_name 파라미터를 직접 사용 (여러 개라면 규칙 필요, 여기서는 단일 파일 기준)
        std::string current_file = (lane_config.load_count == 1) ? file_name : std::to_string(std::stoi(file_name) + i);
        std::string filename = current_file + ".json";
        
        std::string file_path = base_dir + filename;
        std::ifstream f(file_path);
        if (!f.is_open()) {
            ROS_ERROR("Failed to open: %s", file_path.c_str());
            continue;
        }

        json data;
        try { f >> data; } catch (...) { continue; }

        const auto& ids = data["roadgraph_samples/id"];
        const auto& xyz_flat = data["roadgraph_samples/xyz"];
        const auto& valids = data["roadgraph_samples/valid"];
        const auto& dir_flat = data["roadgraph_samples/dir"];
        const auto& type = data["roadgraph_samples/type"];
        const auto& explicit_vals = data["roadgraph_samples/explicit"];
        // const auto& types = data["roadgraph_samples/type"]; // 필요 시 사용

        std::set<int> target_types = {6,7,8,9,10,11,12,13};

        size_t num_points = ids.size();

        for (size_t k = 0; k < num_points; ++k) {
            if (valids[k] != 1) continue;

            int current_type = (int)type[k];
            if (target_types.find(current_type) == target_types.end()) continue;

            int id = ids[k];

            if (global_map.find(id) == global_map.end()) {
                Lane new_lane; new_lane.id = id; 
                new_lane.type = current_type;
                global_map[id] = new_lane;
                new_lane.explicit_lane = explicit_vals[k];
            }
            Point6D pt;
            pt.x = xyz_flat[3 * k];
            pt.y = xyz_flat[3 * k + 1];
            pt.z = xyz_flat[3 * k + 2];
            pt.dx = dir_flat[3 * k];
            pt.dy = dir_flat[3 * k + 1];
            pt.dz = dir_flat[3 * k + 2];

            global_map[id].points.push_back(pt);
        }
    }
    ROS_INFO("Original Total Lanes: %lu", global_map.size());

    // ==========================================
    // [수정된 부분] 맵 자르기 적용
    // ==========================================
    std::map<int, Lane>* map_to_save = &global_map;
    std::map<int, Lane> cropped_map; // 스택에 선언

    if (crop_mode) {
        // 300m x 300m 크기로 자르기 (원하는 크기로 조절 가능)
        cropped_map = filterDenseRegion(global_map, 300.0);
        map_to_save = &cropped_map;
        ROS_INFO("Filtered Lanes: %lu (Saved only dense area)", map_to_save->size());
    }

    std::string output_file = output_dir + "points_seq_0.bin";
    saveToBin(output_file, *map_to_save);
    
    ROS_INFO("Saved to %s", output_file.c_str());

    // ==========================================
    ROS_INFO(">>>Reordering points in each lane...");
    for (auto& pair : *map_to_save) {
        LaneUtils::ReorderPoints(pair.second);
    }
    ROS_INFO(">>>Reordering completed.");
    std::string reordered_output_file = output_dir + "points_seq_1.bin";
    saveToBin(reordered_output_file, *map_to_save);
    ROS_INFO("Saved reordered lanes to %s", reordered_output_file.c_str());


    // ==========================================
    ROS_INFO(">>>Cleaning overlapping lane ends...");
    LaneCleaner::TrimOverlappingLaneEnds(*map_to_save, lane_config);
    ROS_INFO(">>>Cleaning completed.");
    std::string cleaned_output_file = output_dir + "points_seq_2.bin";
    saveToBin(cleaned_output_file, *map_to_save);
    ROS_INFO("Saved cleaned lanes to %s", cleaned_output_file.c_str());

    // ==========================================
    ROS_INFO(">>>Merging fragmented lanes...");
    LaneMerger::MergeFragmentedLanes(*map_to_save, lane_config);
    ROS_INFO(">>>Merging completed.");
    std::string merged_output_file = output_dir + "points_seq_3.bin";
    saveToBin(merged_output_file, *map_to_save);
    ROS_INFO("Saved merged lanes to %s", merged_output_file.c_str());

    return 0;
}