#include <common/io.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <realtime_line_generator/MapConverterV1.h>
#include <realtime_line_generator/MapConverterV2.h>
#include <realtime_line_generator/realtime_line_builder.h>
#include <real_time_map/CoordinateConverterV1.h>
#include <real_time_map/CoordinateConverterV2.h>
#include <evaluator/realtime_evaluator.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
namespace ldb = linemapdraft_builder;

class RealtimeBuilderNode {

public:
    RealtimeBuilderNode() : nh_("~"), builder_(nh_), evaluator_(nh_) {
        nh_.param<std::string>("input_topic", input_topic_, "/points_raw");
        nh_.param<std::string>("output_folder", output_folder_, "data/issue/converted_bin/");
        nh_.param("file_idx", file_idx_, 20000);
        nh_.param("start_frame", frame_index_, 0);
        nh_.param("playback_rate", playback_rate_, 10.0);
        nh_.param("use_pred_frames", use_pred_frames_, false);
        nh_.param<std::string>("frame_id", frame_id_, "map");
        int pipeline_version = 0;
        if (nh_.getParam("pipeline_version", pipeline_version)) {
            map_converter_version_ = pipeline_version;
            coordinate_converter_version_ = pipeline_version;
        } else {
            nh_.param("coordinate_converter_version", coordinate_converter_version_, 2);
            nh_.param<int>("map_converter_version", map_converter_version_, 1);
        }

        std::string pkg_path = ros::package::getPath("realtime_line_generator");
        normalizeFolder(output_folder_);
        output_root_ = pkg_path + "/" + output_folder_ + std::to_string(file_idx_) + "/";
        pred_frames_dir_ = output_root_ + "pred_frames/";

        frames_dir_ = output_root_ + "frames/";
        voxels_dir_ = output_root_ + "voxels/";
        polylines_dir_ = output_root_ + "polylines/";
        merged_polylines_dir_ = output_root_ + "merged_polylines/";

        gt_path_ = output_root_ + "gt.bin";
        traj_path_ = output_root_ + "vehicle_trajectory.bin";

        if (fs::exists(output_root_)) {
            fs::remove_all(output_root_);
            ROS_INFO("Cleared existing output directory: %s", output_root_.c_str());
        }

        createDirectories();

        if (!use_pred_frames_) {
            sub_ = nh_.subscribe(input_topic_, 1, &RealtimeBuilderNode::cloudCallback, this);
        }

        ROS_INFO("Realtime builder output root: %s", output_root_.c_str());
        if (use_pred_frames_) {
            ROS_INFO("Pred-frames input enabled: %s", pred_frames_dir_.c_str());
        }

        if (!use_pred_frames_) {
            map_converter_thread_ = std::thread([this]() {
                if(map_converter_version_ == 1) {
                  MapConverterV1 converter;
                  converter.run();
                } else {
                  MapConverterV2 converter;
                  converter.run();
                }
            });

            coordinate_converter_thread_ = std::thread([this]() {
                if (coordinate_converter_version_ == 1) {
                    CoordinateConverterV1 converter;
                    converter.run();
                    return;
                }
                CoordinateConverterV2 converter;
                converter.run();
            });
        }
    }

    ~RealtimeBuilderNode() {
        if (map_converter_thread_.joinable()) {
            map_converter_thread_.join();
        }
        if (coordinate_converter_thread_.joinable()) {
            coordinate_converter_thread_.join();
        }
    }

    void run() {
        if (!use_pred_frames_) {
            ros::spin();
            logSaveSummary();
            builder_.logTimingSummary();
            return;
        }

        if (map_converter_version_ == 1) {
            MapConverterV1 map_converter;
            map_converter.run();
        } else {
            MapConverterV2 map_converter;
            map_converter.run();
        }
        if (coordinate_converter_version_ == 1) {
            CoordinateConverterV1 coordinate_converter;
            coordinate_converter.run();
        } else {
            CoordinateConverterV2 coordinate_converter;
            coordinate_converter.run();
        }

        end_frame_ = maxFrameIndexInDir(pred_frames_dir_);
        if (end_frame_ < frame_index_) {
            ROS_WARN("No pred_frames found under %s", pred_frames_dir_.c_str());
            return;
        }

        ros::Rate loop_rate(playback_rate_);
        int current_frame = frame_index_;

        while (ros::ok() && current_frame <= end_frame_) {
            std::vector<linemapdraft_builder::data_types::Point> points;
            const std::string frame_path = pred_frames_dir_ + "frame_" + std::to_string(current_frame) + ".bin";

            if (linemapdraft_builder::io::load_points(frame_path, points)) {
                builder_.processPipelineFrame(points, frame_id_);
                saveOutputs(current_frame);
            } else {
                ROS_DEBUG("Pred frame not found, skipping: %s", frame_path.c_str());
            }

            current_frame++;
            ros::spinOnce();
            loop_rate.sleep();
        }

        ROS_INFO("Phase 1 Complete. Starting Phase 2: Evaluation...");
        performEvaluation();

        logSaveSummary();
        builder_.logTimingSummary();
    }

    void performEvaluation() {
        // 저장된 merged_polylines 폴더를 스캔하여 하나씩 평가
        if (!evaluator_.init(gt_path_, traj_path_)) {
            ROS_ERROR("[Evaluator] Failed to load GT or Trajectory files for evaluation!");
            ROS_ERROR("Path: %s, %s", gt_path_.c_str(), traj_path_.c_str());
            return;
        }

        for (int i = frame_index_; i <= end_frame_; ++i) {
            std::vector<std::vector<ldb::data_types::Point>> polylines;
            std::string path = merged_polylines_dir_ + "frame_" + std::to_string(i) + ".bin";

            if (ldb::io::load_polylines(path, polylines)) {
                // 이제 vehicle_trajectory_가 채워져 있으므로 정상 작동합니다.
                evaluator_.evaluateFrame(i, polylines, frame_id_);

                std::string fp_dir = output_root_ + "eval_fp/";
                std::string fn_dir = output_root_ + "eval_fn/";
                evaluator_.saveEvaluationResults(i, fp_dir, fn_dir);
                
                if (i % 50 == 0) ROS_INFO("Evaluating... Frame %d/%d", i, end_frame_);
            }
        }
        evaluator_.printFinalSummary();
    }

private:
    void normalizeFolder(std::string& folder) {
        if (!folder.empty() && folder.back() != '/') {
            folder.push_back('/');
        }
    }

    void createDirectories() {
        if (!use_pred_frames_) {
            fs::create_directories(frames_dir_);
        }
        fs::create_directories(voxels_dir_);
        fs::create_directories(polylines_dir_);
        fs::create_directories(merged_polylines_dir_);
        fs::create_directories(output_root_ + "eval_fp/");
        fs::create_directories(output_root_ + "eval_fn/");
    }

    std::string framePath(const std::string& dir, int index) const {
        return dir + "frame_" + std::to_string(index) + ".bin";
    }

    int maxFrameIndexInDir(const std::string& dir) const {
        if (!fs::exists(dir)) {
            return -1;
        }
        int max_index = -1;
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            const auto& path = entry.path();
            if (path.extension() != ".bin") {
                continue;
            }
            const std::string stem = path.stem().string();
            const std::string prefix = "frame_";
            if (stem.rfind(prefix, 0) != 0) {
                continue;
            }
            const std::string index_str = stem.substr(prefix.size());
            try {
                int idx = std::stoi(index_str);
                if (idx > max_index) {
                    max_index = idx;
                }
            } catch (...) {
            }
        }
        return max_index;
    }

    std::size_t countPolylinePoints(
        const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines) const {
        std::size_t total = 0;
        for (const auto& polyline : polylines) {
            total += polyline.size();
        }
        return total;
    }

    void logSaveSummary() const {
        if (use_pred_frames_) {
            ROS_INFO("Frames output skipped (use_pred_frames=true).");
        } else {
            ROS_INFO("Saved frames: %zu files (%zu points).", saved_frame_files_, saved_frame_points_);
        }
        ROS_INFO("Saved voxels: %zu files (%zu points).", saved_voxel_files_, saved_voxel_points_);
        ROS_INFO("Saved polylines: %zu files (%zu polylines, %zu points).",
                 saved_polyline_files_, saved_polylines_, saved_polyline_points_);
        ROS_INFO("Saved merged polylines: %zu files (%zu polylines, %zu points).",
                 saved_merged_polyline_files_, saved_merged_polylines_, saved_merged_polyline_points_);
    }

    void saveOutputs(int output_index) {
        const auto& active_voxels = builder_.getLastActiveVoxels();
        const auto& polylines = builder_.getLastPolylines();
        const auto& merged_polylines = builder_.getLastMergedPolylines();

        const std::string frame_path = framePath(frames_dir_, output_index);
        const std::string voxel_path = framePath(voxels_dir_, output_index);
        const std::string polyline_path = framePath(polylines_dir_, output_index);
        const std::string merged_path = framePath(merged_polylines_dir_, output_index);

        if (!use_pred_frames_) {
            const auto& frame_points = builder_.getLastFramePoints();
            if (!linemapdraft_builder::io::write_points(frame_path, frame_points)) {
                ROS_WARN("Failed to write frame points: %s", frame_path.c_str());
            } else {
                saved_frame_files_++;
                saved_frame_points_ += frame_points.size();
            }
        }
        if (!linemapdraft_builder::io::write_points(voxel_path, active_voxels)) {
            ROS_WARN("Failed to write voxels: %s", voxel_path.c_str());
        } else {
            saved_voxel_files_++;
            saved_voxel_points_ += active_voxels.size();
        }
        if (!linemapdraft_builder::io::write_polylines(polyline_path, polylines)) {
            ROS_WARN("Failed to write polylines: %s", polyline_path.c_str());
        } else {
            saved_polyline_files_++;
            saved_polylines_ += polylines.size();
            saved_polyline_points_ += countPolylinePoints(polylines);
        }
        if (!linemapdraft_builder::io::write_polylines(merged_path, merged_polylines)) {
            ROS_WARN("Failed to write merged polylines: %s", merged_path.c_str());
        } else {
            saved_merged_polyline_files_++;
            saved_merged_polylines_ += merged_polylines.size();
            saved_merged_polyline_points_ += countPolylinePoints(merged_polylines);
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        builder_.callback(msg);
        saveOutputs(frame_index_);
        frame_index_++;
    }

    ros::NodeHandle nh_;
    realtime_line_generator::RealTimeLineBuilder builder_;
    realtime_line_generator::RealTimeEvaluator evaluator_;
    ros::Subscriber sub_;
    std::string input_topic_;
    std::string output_folder_;
    std::string output_root_;
    std::string pred_frames_dir_;
    std::string frames_dir_;
    std::string voxels_dir_;
    std::string polylines_dir_;
    std::string merged_polylines_dir_;
    std::string gt_path_;
    std::string traj_path_;
    std::string frame_id_;
    int file_idx_;
    int frame_index_;
    int end_frame_ = -1;
    double playback_rate_;
    bool use_pred_frames_;
    int coordinate_converter_version_;
    int map_converter_version_;
    std::thread map_converter_thread_;
    std::thread coordinate_converter_thread_;

    std::size_t saved_frame_files_ = 0;
    std::size_t saved_voxel_files_ = 0;
    std::size_t saved_polyline_files_ = 0;
    std::size_t saved_merged_polyline_files_ = 0;
    std::size_t saved_frame_points_ = 0;
    std::size_t saved_voxel_points_ = 0;
    std::size_t saved_polyline_points_ = 0;
    std::size_t saved_merged_polyline_points_ = 0;
    std::size_t saved_polylines_ = 0;
    std::size_t saved_merged_polylines_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_builder_node");
    RealtimeBuilderNode node;
    node.run();
    return 0;
}
