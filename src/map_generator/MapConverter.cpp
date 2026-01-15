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
    base_dir_ = pkg_path + "/data/lane_change_data/" + input_folder_name_;
    output_dir_ = pkg_path + "/data/lane_change_data_converted/" + output_folder_name_;

    // 출력 디렉토리 생성
    struct stat st = {0};
    if (stat(output_dir_.c_str(), &st) == -1) {
         std::string cmd = "mkdir -p " + output_dir_;
         system(cmd.c_str());
    }
}

void MapConverter::run() {
    ROS_INFO("========================================");
    ROS_INFO(">>> STARTING MAP CONVERTER");
    ROS_INFO("========================================");

    // 파일 로딩
    std::string file_path = base_dir_ + "";


    std::vector<std::string> file_names;

    try {
        if (fs::exists(base_dir_) && fs::is_directory(base_dir_)) {
            // directory_iterator를 사용하여 반복 수행
            for (const auto& entry : fs::directory_iterator(base_dir_)) {
                // 디렉토리가 아닌 '파일'인 경우만 이름 추가
                if (fs::is_regular_file(entry)) {
                    file_names.push_back(entry.path().filename().string());
                }
            }
        }
    } catch (const fs::filesystem_error& err) {
        std::cerr << "에러 발생: " << err.what() << std::endl;
    }


    for (const auto& name : file_names) {
        std::string file_path = base_dir_ + name;

        std::ifstream f(file_path);

        ROS_INFO("Loading file: %s", file_path.c_str());

        global_map_.clear();

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
                        Lane new_lane; new_lane.id = id; 
                        new_lane.type = (int)type[k];
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

        fs::path p(name);

        // 2. .json 확장자를 가진 파일인지 확인 후 처리
        if (p.extension() == ".json") {
            // 3. 확장자를 제외한 이름 추출 (예: "001.json" -> "001")
            std::string folder_name = p.stem().string();

            // 4. 저장할 디렉토리 경로 
            std::string target_dir = output_dir_ + folder_name + "/";

            // 5. 디렉토리가 없으면 생성
            if (!fs::exists(target_dir)) {
                fs::create_directories(target_dir);
            }

            saveToBin(target_dir + "points_seq_0.bin", *map_to_save);

            // ROS_INFO(">>> Reordering...");
            // for (auto& pair : *map_to_save) LaneUtils::ReorderPoints(pair.second);
            // saveToBin(target_dir + "points_seq_1.bin", *map_to_save);

            // ROS_INFO(">>> Cleaning...");
            // LaneCleaner::TrimOverlappingLaneEnds(*map_to_save, lane_config_);
            // saveToBin(target_dir + "points_seq_2.bin", *map_to_save);

            // ROS_INFO(">>> Merging...");
            // LaneMerger::MergeFragmentedLanes(*map_to_save, lane_config_);
            // saveToBin(target_dir + "points_seq_3.bin", *map_to_save);
            
            ROS_INFO(">>> MAP CONVERTER FINISHED.");

        }
    }
}