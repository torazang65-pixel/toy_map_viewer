
#include <common/data_types.h>
#include <common/io.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <filesystem>
#include <fstream>
#include <vector>

namespace linemapdraft_builder::io {

bool load_points(const std::string &filename, std::vector<data_types::Point> &points) {
  ROS_DEBUG("Loading points...");
  points.clear();
  std::ifstream ifs(filename, std::ifstream::binary);
  if (!ifs.is_open()) {
    ROS_ERROR("Failed to open input voxel file: %s", filename.c_str());
    return false;
  }

  uint32_t num_points = 0;
  ifs.read((char *)&num_points, 4);
  points.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    float x, y, z, yaw;
    int id;
    uint32_t density;
    ifs.read((char *)&x, 4);
    ifs.read((char *)&y, 4);
    ifs.read((char *)&z, 4);
    ifs.read((char *)&yaw, 4);
    ifs.read((char *)&id, 4);
    ifs.read((char *)&density, 4);

    points[i] = {x, y, z, yaw, .0f, id, density};
  }

  ifs.close();
  ROS_DEBUG("Loaded %u points from %s", num_points, filename.c_str());
  return true;
}

bool write_points(const std::string &filename, const std::vector<data_types::Point> &points) {
  std::ofstream ofs(filename, std::ofstream::binary);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open output file: %s", filename.c_str());
    return false;
  }

  uint32_t num_points = static_cast<uint32_t>(points.size());
  ofs.write((char *)(&num_points), 4);

  for (const auto &p : points) {
    ofs.write((char *)(&p.x), 4);
    ofs.write((char *)(&p.y), 4);
    ofs.write((char *)(&p.z), 4);
    ofs.write((char *)(&p.yaw), 4);
    ofs.write((char *)(&p.polyline_id), 4);
    ofs.write((char *)(&p.density), 4);
  }

  ofs.close();
  ROS_DEBUG("Wrote %u points to %s", num_points, filename.c_str());
  return true;
}

bool load_polylines(const std::string &filename, std::vector<std::vector<data_types::Point>> &polylines) {
  std::ifstream polylines_ifs(filename, std::ios::binary);
  if (!polylines_ifs.is_open()) {
    ROS_ERROR("Failed to open input file: %s", filename.c_str());
    return false;
  }

  // Step 1. read total polyline number;
  uint32_t polyline_num;
  polylines_ifs.read((char *)(&polyline_num), 4);
  polylines.resize(polyline_num);

  // For each polyline
  for (uint32_t i = 0; i < polyline_num; i++) {
    // Step 2. polyline id
    int32_t current_polyline_id;
    polylines_ifs.read((char *)(&current_polyline_id), 4);
    // Step 3. layer
    int32_t current_polyline_layer;
    polylines_ifs.read((char *)(&current_polyline_layer), 4);
    // Step 4. polyline size
    uint32_t current_polyline_size;
    polylines_ifs.read((char *)(&current_polyline_size), 4);

    // Step 5. read each point(x, y, z) in polyline
    for (uint32_t j = 0; j < current_polyline_size; j++) {
      float x, y, z;
      polylines_ifs.read((char *)(&x), 4);
      polylines_ifs.read((char *)(&y), 4);
      polylines_ifs.read((char *)(&z), 4);
      polylines[current_polyline_id].push_back({x, y, z});
    }
  }

  polylines_ifs.close();
  ROS_DEBUG("Loaded %u polylines from %s", polyline_num, filename.c_str());
  return true;
}

bool write_polylines(const std::string &filename, const std::vector<std::vector<data_types::Point>> &polylines) {
  std::ofstream ofs(filename, std::ofstream::binary);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open output file: %s", filename.c_str());
    return false;
  }

  // Step 1. write total polyline number
  uint32_t out_polyline_num = static_cast<uint32_t>(polylines.size());
  ofs.write((char *)(&out_polyline_num), 4);

  // For each polyline
  int32_t current_polyline_id = 0;
  for (const auto &polyline : polylines) {
    // Step 2. polyline id
    ofs.write((char *)(&current_polyline_id), 4);
    current_polyline_id++;
    // Step 3. layer
    ofs.write("\0\0\0\0", 4);
    // Step 4. polyline size
    uint32_t current_polyline_size = static_cast<uint32_t>(polyline.size());
    ofs.write((char *)(&current_polyline_size), 4);
    // Step 5. write each point(x, y, z) in polyline
    for (const auto &p : polyline) {
      ofs.write((char *)&p.x, 4);
      ofs.write((char *)&p.y, 4);
      ofs.write((char *)&p.z, 4);
    }
  }

  ofs.close();
  ROS_DEBUG("Wrote %u polylines to %s", current_polyline_id, filename.c_str());
  return true;
}

bool write_points_from_polylines(const std::string &filename, const std::vector<std::vector<data_types::Point>> &polylines) {
  std::ofstream ofs(filename, std::ofstream::binary);
  int out_point_num = 0;

  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open output file: %s", filename.c_str());
    return false;
  }

  ofs.seekp(4);  // Reserve space for the number of points
  for (const auto &polyline : polylines) {
    for (const auto &p : polyline) {
      ofs.write((char *)&p.x, 4);
      ofs.write((char *)&p.y, 4);
      ofs.write((char *)&p.z, 4);
      ofs.write((char *)&p.yaw, 4);
      ofs.write((char *)&p.polyline_id, 4);
      ofs.write((char *)&p.density, 4);

      out_point_num++;
    }
  }
  ofs.seekp(0);
  ofs.write((char *)&out_point_num, 4);  // Write the number of points at the beginning

  ofs.close();
  ROS_DEBUG("done. (%s)", filename.c_str());
  ROS_DEBUG("out_point_num: %d", out_point_num);
  return true;
}

}  // namespace linemapdraft_builder::io
