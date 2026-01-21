#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <cmath>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace linemapdraft_builder::data_types {

// gt 생성용 새로운 struct 정의.
struct Point6D {
  double x, y, z;
  double dx, dy, dz;
};

struct Lane {
  int id;
  int type;
  bool valid;
  bool explicit_lane;
  std::vector<Point6D> points;
};

// original linemapdraft_builder start here
constexpr int full_seq_size = 0;  // default value for full sequence size

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

struct RTreeIndex {
  size_t polyline_idx;
  size_t point_idx;

  RTreeIndex(size_t _polyline_idx, size_t _point_idx) : polyline_idx(_polyline_idx), point_idx(_point_idx) {}
  bool operator==(const RTreeIndex& rhs) const {
    return polyline_idx == rhs.polyline_idx && point_idx == rhs.point_idx;
  }
};

typedef bg::model::point<double, 2, bg::cs::cartesian> bg_point;
typedef bg::model::segment<bg_point> bg_segment;

typedef std::pair<bg_point, RTreeIndex> point_value;
typedef bgi::rtree<point_value, bgi::quadratic<16>> point_rtree;

using GridKey = std::pair<int, int>;
using GridMap = std::map<GridKey, std::vector<int>>;

enum PolylineState {
  Unclassified = -1,
  Dropped = -2,
};

struct Point {
  float x, y, z;
  float yaw;
  float vz;
  int32_t polyline_id = Unclassified;
  uint32_t density = 1;

  Point operator+(const Point& rhs) const {
    return Point{
        x + rhs.x,
        y + rhs.y,
        z + rhs.z,
        yaw,
        vz,
        Unclassified};
  }

  Point operator-(const Point& rhs) const {
    return Point{
        x - rhs.x,
        y - rhs.y,
        z - rhs.z,
        yaw,
        vz,
        Unclassified};
  }

  Point operator*(float s) const {
    return {x * s, y * s, z * s, yaw, vz, Unclassified};
  }

  float norm() const {
    return std::sqrt(x * x + y * y + z * z);
  }

  Point normalized() const {
    float n = norm();
    if (n == 0.0f) return *this;
    return (*this) * (1.0f / n);
  }

  float dot(const Point& rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
  }

  Point cross(const Point& rhs) const {
    return Point{y * rhs.z - z * rhs.y,
                 z * rhs.x - x * rhs.z,
                 x * rhs.y - y * rhs.x,
                 yaw, vz, Unclassified};
  }

  friend Point operator*(float s, const Point& p) {
    return p * s;
  }
};

Point computeAveragePoint(
    const std::vector<Point>& points,
    const std::vector<int>& idxs);

struct PolylineData {
  std::vector<Point> points;
  Point A;
  Point B;
  bool valid = true;

  void updateEndpoints() {
    if (!points.empty()) {
      A = points.front();
      B = points.back();
    }
  }
};

void build_grid_map(const std::vector<Point>& points, float grid_size, GridMap& grid_map);
std::vector<int> getNeighborIndices(const std::vector<Point>& points,
                                    const GridMap& grid,
                                    float grid_size,
                                    const Point& center);

void build_rtree(point_rtree& rtree, const std::vector<std::vector<data_types::Point>>& polylines);
void removeIndex(point_rtree& rtree, size_t id, const std::vector<data_types::Point>& curve);
void insertIndex(point_rtree& rtree, size_t id, const std::vector<data_types::Point>& curve);
std::optional<data_types::point_value> queryUniquePointValue(const data_types::point_rtree& rtree, const float x, const float y);

}  // namespace linemapdraft_builder::data_types
