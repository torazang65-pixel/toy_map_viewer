#include <common/data_types.h>

#include <filesystem>
#include <fstream>
#include <vector>

namespace linemapdraft_builder::io {

bool load_points(const std::string &filename, std::vector<data_types::Point> &points);

bool write_points(const std::string &filename, const std::vector<data_types::Point> &points);

bool load_polylines(const std::string &filename, std::vector<std::vector<data_types::Point>> &polylines);

bool write_polylines(const std::string &filename, const std::vector<std::vector<data_types::Point>> &polylines);

bool write_points_from_polylines(const std::string &filename, const std::vector<std::vector<data_types::Point>> &polylines);
}  // namespace linemapdraft_builder::io
