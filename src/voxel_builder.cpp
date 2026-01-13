#include <linemapdraft_builder/voxel_builder.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <common/data_types.h>
#include <common/io.h>
#include <common/path_manager.h>

#include <array>
#include <boost/geometry.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <unordered_map>

namespace bg = boost::geometry;
namespace fs = std::filesystem;

namespace linemapdraft_builder::voxel_builder {
typedef bg::model::point<float, 2, bg::cs::cartesian> bg_point;
typedef bg::model::polygon<bg_point> bg_polygon;

using VoxelKey = std::array<int, 4>;

struct VoxelAccum {
  int count = 0;
  double sum_x = 0, sum_y = 0, sum_z = 0;
  double sum_yaw = 0;
};

struct ArrayHasher {
  std::size_t operator()(const std::array<int, 4> &key) const {
    return std::hash<int>()(key[0]) * 73856093 ^
           std::hash<int>()(key[1]) * 19349663 ^
           std::hash<int>()(key[2]) * 83492791 ^
           std::hash<int>()(key[3]) * 2971215073;
  }
};

bool build(const std::string &file_path) {
  ros::NodeHandle nh("~");

  // 데이터 로드
  std::vector<data_types::Point> points;
  for (const auto& path : input_paths) {
      std::vector<data_types::Point> frame_points;
      if (linemapdraft_builder::io::load_points(path, frame_points)) {
          points.insert(points.end(), frame_points.begin(), frame_points.end());
      }
  }

  if (points.empty()) return false;


  // Get parameters from ROS parameter server
  float voxel_size;
  int yaw_voxel_num;
  bool partial_gt_mode;
  std::vector<std::vector<int>> partial_gt_area;

  nh.param("voxel_size", voxel_size, 0.5f);
  nh.param("yaw_voxel_num", yaw_voxel_num, 1);
  nh.param("partial_gt_mode", partial_gt_mode, false);

  XmlRpc::XmlRpcValue raw_partial_gt_area;
  if (nh.getParam("partial_gt_area", raw_partial_gt_area)) {
    if (raw_partial_gt_area.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < raw_partial_gt_area.size(); ++i) {
        if (raw_partial_gt_area[i].getType() == XmlRpc::XmlRpcValue::TypeArray &&
            raw_partial_gt_area[i].size() == 2 &&
            raw_partial_gt_area[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt &&
            raw_partial_gt_area[i][1].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          partial_gt_area.push_back({static_cast<int>(raw_partial_gt_area[i][0]),
                                     static_cast<int>(raw_partial_gt_area[i][1])});
        } else {
          ROS_WARN("Invalid point format at index %d", i);
        }
      }
      ROS_INFO("Loaded partial_gt_area with %lu points", partial_gt_area.size());
    } else {
      ROS_WARN("partial_gt_area is not an array");
    }
  } else {
    ROS_WARN("partial_gt_area param not found");
  }

  const float yaw_voxel_size = M_PI / yaw_voxel_num;

  ROS_INFO("voxel build start.");
  ROS_INFO("voxel_size: %f, yaw_voxel_size:%f", voxel_size, yaw_voxel_size);

  const std::string line_pc_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::LinePC, PathManager::BinaryKind::Points);
  const std::string line_voxel_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::LineVoxel, PathManager::BinaryKind::Points);

  uint32_t in_point_num;
  std::ifstream line_pc_ifs(line_pc_filename, std::ifstream::binary);
  std::ofstream line_voxel_ofs(line_voxel_filename, std::ofstream::binary);
  if (!line_pc_ifs.is_open()) {
    ROS_ERROR("Failed to open line pc file. (%s)", line_pc_filename.c_str());
    return false;
  }

  if (!line_voxel_ofs.is_open()) {
    ROS_ERROR("Failed to open line voxel file. (%s)", line_voxel_filename.c_str());
    line_pc_ifs.close();
    return false;
  }

  line_pc_ifs.read((char *)&in_point_num, 4);

  std::unordered_map<VoxelKey, VoxelAccum, ArrayHasher> voxel_map;

  for (uint32_t i = 0; i < in_point_num; i++) {
    float x, y, z, yaw;
    int dummy_id;
    uint32_t dummy_density;
    line_pc_ifs.read((char *)&x, 4);
    line_pc_ifs.read((char *)&y, 4);
    line_pc_ifs.read((char *)&z, 4);
    line_pc_ifs.read((char *)&yaw, 4);
    line_pc_ifs.read((char *)&dummy_id, 4);
    line_pc_ifs.read((char *)&dummy_density, 4);

    int vx = static_cast<int>(std::floor(x / voxel_size));
    int vy = static_cast<int>(std::floor(y / voxel_size));
    int vz = static_cast<int>(std::floor(z / voxel_size));
    int vyaw = static_cast<int>(std::floor(yaw / yaw_voxel_size));

    VoxelKey key = {vx, vy, vz, vyaw};

    auto &acc = voxel_map[key];
    acc.count++;
    acc.sum_x += x;
    acc.sum_y += y;
    acc.sum_z += z;
    acc.sum_yaw += yaw;
  }

  uint32_t out_point_num = 0;
  line_voxel_ofs.seekp(4);

  if (partial_gt_area.size() != 4) {
    ROS_ERROR("Partial GT area must have exactly 4 points.");
    return false;
  }

  for (const auto &point : partial_gt_area) {
    std::cout << "Partial GT Area Point: (" << point[0] << ", " << point[1] << ")" << std::endl;
  }

  bg_polygon partial_gt_area_polygon = bg_polygon(
      {{bg_point{static_cast<float>(partial_gt_area[0][0]), static_cast<float>(partial_gt_area[0][1])},
        bg_point{static_cast<float>(partial_gt_area[1][0]), static_cast<float>(partial_gt_area[1][1])},
        bg_point{static_cast<float>(partial_gt_area[2][0]), static_cast<float>(partial_gt_area[2][1])},
        bg_point{static_cast<float>(partial_gt_area[3][0]), static_cast<float>(partial_gt_area[3][1])},
        bg_point{static_cast<float>(partial_gt_area[0][0]), static_cast<float>(partial_gt_area[0][1])}}});
  
  std::vector<uint32_t> cnt;
  cnt.reserve(voxel_map.size());
  for(const auto &[key, acc]: voxel_map){
    if(acc.count > 1) cnt.push_back(acc.count);
  }
  std::sort(cnt.begin(), cnt.end());
  auto m = cnt.size();
  auto Q1 = cnt[m / 4]; 
  for (const auto &[key, acc] : voxel_map) {
    if (acc.count <= Q1) continue;

    float x = acc.sum_x / acc.count;
    float y = acc.sum_y / acc.count;
    float z = acc.sum_z / acc.count;
    float yaw = acc.sum_yaw / acc.count;
    int id = -1;
    uint32_t density = acc.count;

    while (yaw < 0) yaw += 2 * M_PI;
    while (yaw >= M_PI) yaw -= M_PI;

    if (partial_gt_mode) {
      bg_point pt{static_cast<float>(x), static_cast<float>(y)};
      if (!bg::within(pt, partial_gt_area_polygon)) {
        continue;  // Skip points outside the partial GT area
      }
    }

    line_voxel_ofs.write((char *)&x, 4);
    line_voxel_ofs.write((char *)&y, 4);
    line_voxel_ofs.write((char *)&z, 4);
    line_voxel_ofs.write((char *)&yaw, 4);
    line_voxel_ofs.write((char *)&id, 4);
    line_voxel_ofs.write((char *)&density, 4);

    ++out_point_num;
  }

  line_voxel_ofs.seekp(0);
  line_voxel_ofs.write((char *)&out_point_num, 4);

  line_pc_ifs.close();
  line_voxel_ofs.close();
  ROS_INFO("done. (%s)", line_voxel_filename.c_str());
  ROS_INFO("in_point_num: %d, out_point_num: %d", in_point_num, out_point_num);
  return true;
}

}  // namespace linemapdraft_builder::voxel_builder