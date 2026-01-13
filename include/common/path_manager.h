#pragma once

#include <ros/package.h>

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

// todo: namespace 기반으로 좀 쪼개두자
namespace linemapdraft_builder {

class PathManager {
 public:
  struct PathContext {
    int idx = 2;
    int sequence_size = 0;
    std::string district = "SEUL13";
    std::string point_cls = "LaneLine";
  };

  enum class Stage {
    LinePC,
    LineVoxel,
    Polyline,
    SimplePolyline,
    PostProcessed,
    FinalPolyline
  };

  enum class BinaryKind {
    Points,
    Polylines,
    DraftIntersections,
    GTIntersections,
    DraftFPs,
    GTFNs
  };

  static std::string stageToString(Stage s) {
    switch (s) {
      case Stage::LinePC:
        return "LinePC";
      case Stage::LineVoxel:
        return "LineVoxel";
      case Stage::Polyline:
        return "Polyline";
      case Stage::SimplePolyline:
        return "SimplePolyline";
      case Stage::PostProcessed:
        return "PostProcessed";
      case Stage::FinalPolyline:
        return "FinalPolyline";
    }
    return "";
  }

  static std::string binaryKindToString(BinaryKind kind) {
    switch (kind) {
      case BinaryKind::Points:
        return "Points";
      case BinaryKind::Polylines:
        return "Polylines";
      case BinaryKind::DraftIntersections:
        return "DraftIntersections";
      case BinaryKind::GTIntersections:
        return "GTIntersections";
      case BinaryKind::DraftFPs:
        return "DraftFPs";
      case BinaryKind::GTFNs:
        return "GTFNs";
    }
    return "";
  }

  static std::string basePath() {
    return ros::package::getPath("linemapdraft_builder") + "/data";
  }

  static std::string getModelOutputDir(const PathContext &ctx) {
    return basePath() + "/ModelOutputs/" + ctx.district + "/" + ctx.point_cls + "/" + std::to_string(ctx.idx);
  }

  static std::string getPoseDir(const PathContext &ctx) {
    return basePath() + "/Poses/" + ctx.district;
  }

  static std::string getMergeInfoDir(const PathContext &ctx) {
    return basePath() + "/MergeInfos/" + ctx.district;
  }

  static std::string getGTDir(const PathContext &ctx) {
    return basePath() + "/GT/" + ctx.district + "/" + ctx.point_cls;
  }

  static std::string getGTBinaryPath(bool partial_gt_mode, const PathContext &ctx) {
    std::string filename = partial_gt_mode ? "partial" : "full";
    fs::path file = getGTDir(ctx) + "/" + filename + ".bin";
    fs::create_directories(file.parent_path());
    return file.string();
  }

  static std::string getStageDir(const PathContext &ctx, Stage s) {
    return basePath() + "/" + getStageDirName(s) + "/" + ctx.district + "/" + ctx.point_cls + "/" + std::to_string(ctx.idx);
  }

  static std::string getStageBinaryPath(const PathContext &ctx, Stage s, BinaryKind kind) {
    fs::path file = getStageDir(ctx, s) + "/" + getBinaryKindPrefix(kind) + "seq_" + std::to_string(ctx.sequence_size) + ".bin";
    fs::create_directories(file.parent_path());
    return file.string();
  }

 private:
  static std::string getStageDirName(Stage s) {
    switch (s) {
      case Stage::LinePC:
        return "LinePCs";
      case Stage::LineVoxel:
        return "LineVoxels";
      case Stage::Polyline:
        return "Polylines";
      case Stage::SimplePolyline:
        return "SimplePolylines";
      case Stage::PostProcessed:
        return "PostProcessed";
      case Stage::FinalPolyline:
        return "FinalPolylines";
    }

    return "";
  }

  static std::string getBinaryKindPrefix(BinaryKind kind) {
    switch (kind) {
      case BinaryKind::Points:
        return "points_";
      case BinaryKind::Polylines:
        return "polylines_";
      case BinaryKind::DraftIntersections:
        return "draft_intersections_";
      case BinaryKind::GTIntersections:
        return "gt_intersections_";
      case BinaryKind::DraftFPs:
        return "draft_fps_";
      case BinaryKind::GTFNs:
        return "gt_fns_";
    }
    return "";
  }
};

}  // namespace linemapdraft_builder
