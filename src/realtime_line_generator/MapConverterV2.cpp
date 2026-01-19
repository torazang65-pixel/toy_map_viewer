#include "realtime_line_generator/MapConverterV2.h"
#include "common/io.h"

#include <fstream>
#include <set>
#include <utility>
#include <sys/stat.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
namespace ldb = linemapdraft_builder;

MapConverterV2::MapConverterV2() : nh_("~") {
    loadParameters();
}

void MapConverterV2::loadParameters() {
    nh_.param<std::string>("package_name", package_name_, "realtime_line_generator");
    nh_.param<int>("file_idx", lane_config_.file_idx, 0);
    nh_.param<std::string>("input_folder", input_folder_name_, "data/issue/global_maps/");
    nh_.param<std::string>("output_folder", output_folder_name_, "data/issue/converted_bin/");

    std::string pkg_path = ros::package::getPath(package_name_);
    base_dir_ = pkg_path + "/" + input_folder_name_;
    output_dir_ = pkg_path + "/" + output_folder_name_ + std::to_string(lane_config_.file_idx) + "/";

    struct stat st = {0};
    if (stat(output_dir_.c_str(), &st) == -1) {
        std::string cmd = "mkdir -p " + output_dir_;
        system(cmd.c_str());
    }
}

void MapConverterV2::run() {
    int current_idx = lane_config_.file_idx;
    std::string filename = std::to_string(current_idx) + ".json";
    std::string file_path = base_dir_ + filename;

    std::ifstream f(file_path);
    if (!f.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
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
        ROS_ERROR("JSON parsing error: %s", e.what());
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

    const std::string output_path = output_dir_ + "points_seq_0.bin";
    ldb::io::write_points_from_polylines(output_path, polylines);
    ROS_INFO("Saved points_seq_0.bin (%lu lanes).", polylines.size());
}
