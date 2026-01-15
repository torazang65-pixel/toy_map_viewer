#include "toy_map_viewer/MapConverter.h"
#include "toy_map_viewer/BinSaver.h"
#include "toy_map_viewer/LaneCleaner.h"
#include "toy_map_viewer/LaneMerger.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <limits>
#include <cmath>
#include <set>
#include <nlohmann/json.hpp>
#include <filesystem>

using json = nlohmann::json;
namespace fs = std::filesystem;

MapConverter::MapConverter() : nh_("~") {
    loadParameters();
}

void MapConverter::loadParameters() {
    // 파라미터 로딩
    nh_.param<std::string>("package_name", package_name_, "toy_map_viewer");
    nh_.param<std::string>("input_folder", input_folder_name_, "map_prev/");
    nh_.param<std::string>("output_folder", output_folder_name_, "map_prev/");
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
    // 2. 하드코딩 대신 파라미터 변수를 사용하여 경로를 완성합니다.
    base_dir_ = pkg_path + "/data/lane_change_data/" + input_folder_name_;
    output_dir_ = pkg_path + "/data/lane_change_data_converted/" + output_folder_name_;
    raw_dir_ = pkg_path + "/data/lane_change_data/Raw/";

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

void LatLonToUTM(double lat, double lon, double& utm_x, double& utm_y) {
    const double WGS84_A = 6378137.0;
    const double WGS84_F = 1.0 / 298.257223563;
    const double UTM_K0 = 0.9996;
    const double UTM_E2 = WGS84_F * (2.0 - WGS84_F);
    
    double lon0 = 129.0 * M_PI / 180.0; // Zone 52 기준 자오선
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double n = WGS84_A / sqrt(1.0 - UTM_E2 * sin(lat_rad) * sin(lat_rad));
    double t = tan(lat_rad) * tan(lat_rad);
    double c = UTM_E2 * cos(lat_rad) * cos(lat_rad) / (1.0 - UTM_E2);
    double a = (lon_rad - lon0) * cos(lat_rad);

    double m = WGS84_A * ((1.0 - UTM_E2 / 4.0 - 3.0 * UTM_E2 * UTM_E2 / 64.0) * lat_rad
               - (3.0 * UTM_E2 / 8.0 + 3.0 * UTM_E2 * UTM_E2 / 32.0) * sin(2.0 * lat_rad)
               + (15.0 * UTM_E2 * UTM_E2 / 256.0) * sin(4.0 * lat_rad));

    utm_x = UTM_K0 * n * (a + (1.0 - t + c) * a * a * a / 6.0 + (5.0 - 18.0 * t + t * t) * a * a * a * a * a / 120.0) + 500000.0;
    utm_y = UTM_K0 * (m + n * tan(lat_rad) * (a * a / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * a * a * a * a / 24.0
               + (61.0 - 58.0 * t + t * t) * a * a * a * a * a * a / 720.0));
}

void MapConverter::run() {
    ROS_INFO("========================================");
    ROS_INFO(">>> STARTING MAP CONVERTER");

    ROS_INFO(">>> Looking for JSONs in: %s", base_dir_.c_str());
    ROS_INFO(">>> Looking for Raw poses in: %s", raw_dir_.c_str());

    if (lane_config_.crop_mode) ROS_INFO(">>> CROP MODE IS: ON (True)");
    else ROS_INFO(">>> CROP MODE IS: OFF (False)");
    ROS_INFO("========================================");

    std::vector<std::string> file_names;
    try {
        if (fs::exists(base_dir_) && fs::is_directory(base_dir_)) {
            for (const auto& entry : fs::directory_iterator(base_dir_)) {
                if (fs::is_regular_file(entry) && entry.path().extension() == ".json") {
                    file_names.push_back(entry.path().filename().string());
                }
            }
        }
    } catch (const fs::filesystem_error& err) {
        ROS_ERROR("Filesystem error: %s", err.what());
        return;
    }

    for (const auto& name : file_names) {
        std::string file_path = base_dir_ + name;

        ROS_INFO("Reading File in %s", file_path.c_str());
        std::string stem_name = fs::path(name).stem().string();
        std::string global_pose_path = raw_dir_ + stem_name + "/pandar64_1/global_pose/0.json";

        ROS_INFO("Processing file: %s", name.c_str());

        global_map_.clear();

        std::ifstream pose_ifs(global_pose_path);
        if (!pose_ifs.is_open()) {
            ROS_ERROR("포즈 파일을 열 수 없습니다: %s", global_pose_path.c_str());
            continue; 
        }
        
        json j_pose;
        try {
            pose_ifs >> j_pose;
        } catch (const std::exception& e) {
            ROS_ERROR("Pose JSON parsing error: %s", e.what());
            continue;
        }

        // 글로벌 변환 행렬(T_global_map) 생성
        double utm_x, utm_y;
        LatLonToUTM(j_pose["latitude"], j_pose["longitude"], utm_x, utm_y);

        Eigen::Translation3d translation(utm_x, utm_y, j_pose["altitude"]);
        Eigen::Quaterniond rotation(j_pose["q_w"], j_pose["q_x"], j_pose["q_y"], j_pose["q_z"]);
        rotation.normalize(); 
        
        Eigen::Matrix4d T_global_map = (translation * rotation).matrix();
        Eigen::Matrix3d R_global_map = T_global_map.block<3,3>(0,0);

        std::ifstream f(file_path);
        if (f.is_open()){
            json data;
            try {
                f >> data;
                const auto& ids = data["roadgraph_samples/id"];
                const auto& xyz_flat = data["roadgraph_samples/xyz"];
                const auto& valids = data["roadgraph_samples/valid"];
                const auto& dir_flat = data["roadgraph_samples/dir"];
                const auto& type = data["roadgraph_samples/type"];
                
                std::set<int> target_types = {6,7,8,9,10,11,12,13};

                for (size_t k = 0; k < ids.size(); ++k) {
                    if (valids[k] != 1) continue;
                    if (target_types.find((int)type[k]) == target_types.end()) continue;

                    int id = ids[k];
                    if (global_map_.find(id) == global_map_.end()) {
                        Lane new_lane; 
                        new_lane.id = id; 
                        new_lane.type = (int)type[k];
                        global_map_[id] = new_lane;
                    }

                    // 로컬 위치 -> 글로벌 위치 변환
                    Eigen::Vector4d p_local(xyz_flat[3 * k], xyz_flat[3 * k + 1], xyz_flat[3 * k + 2], 1.0);
                    Eigen::Vector4d p_global = T_global_map * p_local;

                    // 로컬 방향 -> 글로벌 방향 변환
                    Eigen::Vector3d d_local(dir_flat[3 * k], dir_flat[3 * k + 1], dir_flat[3 * k + 2]);
                    Eigen::Vector3d d_global = R_global_map * d_local;

                    Point6D pt;
                    pt.x = p_global.x(); pt.y = p_global.y(); pt.z = p_global.z();
                    pt.dx = d_global.x(); pt.dy = d_global.y(); pt.dz = d_global.z();

                    global_map_[id].points.push_back(pt);
                }
                
                ROS_INFO("Global Total Lanes loaded: %lu", global_map_.size());
            } catch (const std::exception& e) {
                ROS_ERROR("JSON parsing error in %s: %s", name.c_str(), e.what());
            }
        } else {
            ROS_ERROR("Failed to open map file: %s", file_path.c_str());
            continue;
        }

        // 처리 및 저장
        std::map<int, Lane>* map_to_save = &global_map_;
        std::map<int, Lane> cropped_map;

        if (lane_config_.crop_mode) {
            cropped_map = filterDenseRegion(global_map_, lane_config_.crop_size);
            map_to_save = &cropped_map;
            ROS_INFO("Filtered Lanes: %lu", map_to_save->size());
        }

        std::string target_dir = output_dir_ + stem_name + "/";
        if (!fs::exists(target_dir)) {
            fs::create_directories(target_dir);
        }

        saveToBin(target_dir + "points_seq_0.bin", *map_to_save);

        ROS_INFO(">>> Reordering %s...", stem_name.c_str());
        for (auto& pair : *map_to_save) LaneUtils::ReorderPoints(pair.second);
        saveToBin(target_dir + "points_seq_1.bin", *map_to_save);

        ROS_INFO(">>> Cleaning %s...", stem_name.c_str());
        LaneCleaner::TrimOverlappingLaneEnds(*map_to_save, lane_config_);
        saveToBin(target_dir + "points_seq_2.bin", *map_to_save);

        ROS_INFO(">>> Merging %s...", stem_name.c_str());
        LaneMerger::MergeFragmentedLanes(*map_to_save, lane_config_);
        saveToBin(target_dir + "points_seq_3.bin", *map_to_save);
        
        ROS_INFO(">>> Finished processing: %s", stem_name.c_str());
    }
}