#pragma once

#include <common/data_types.h>

#include <cmath>

namespace linemapdraft_builder::geometry_utils {

inline float calculate_dist(const data_types::Point& P, const data_types::Point& Q) {
  float dx = P.x - Q.x;
  float dy = P.y - Q.y;
  float dz = P.z - Q.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// calculate yaw of heading vector P -> Q
inline float calculate_heading(const data_types::Point& P, const data_types::Point& Q) {
  float dx = Q.x - P.x;
  float dy = Q.y - P.y;
  return std::atan2(dy, dx);
}

// Compute the shortest (perpendicular) distance from point P to the line segment AB in 3D,
// and set inside_segment to true if the projection of P onto AB lies between A and B.
inline float perpendicular_distance(const data_types::Point& A,
                                    const data_types::Point& B,
                                    const data_types::Point& P,
                                    bool& inside_segment) {
  // Vector AB
  float vx = B.x - A.x;
  float vy = B.y - A.y;
  float vz = B.z - A.z;
  // Vector AP
  float wx = P.x - A.x;
  float wy = P.y - A.y;
  float wz = P.z - A.z;

  // Squared length of AB
  float v_len2 = vx * vx + vy * vy + vz * vz;
  // Projection parameter t = (AP · AB) / |AB|^2
  float dot = vx * wx + vy * wy + vz * wz;
  float t = dot / v_len2;
  inside_segment = (t >= 0.0f && t <= 1.0f);

  // Coordinates of projection point of P onto AB
  float proj_x = t * vx;
  float proj_y = t * vy;
  float proj_z = t * vz;

  // Perpendicular vector components
  float perp_x = wx - proj_x;
  float perp_y = wy - proj_y;
  float perp_z = wz - proj_z;

  // Return its length
  return std::sqrt(perp_x * perp_x +
                   perp_y * perp_y +
                   perp_z * perp_z);
}

// overloading perpendicular_distance
inline float perpendicular_distance(const data_types::Point& A,
                                    const data_types::Point& B,
                                    const data_types::Point& P) {
  bool dummy;
  return perpendicular_distance(A, B, P, dummy);
}

inline float perpendicular_distance_bev(const data_types::Point& A,
                                        const data_types::Point& B,
                                        const data_types::Point& P,
                                        bool& inside_segment) {
  // Vector AB
  float vx = B.x - A.x;
  float vy = B.y - A.y;
  // Vector AP
  float wx = P.x - A.x;
  float wy = P.y - A.y;

  // Squared length of AB
  float v_len2 = vx * vx + vy * vy;
  // Projection parameter t = (AP · AB) / |AB|^2
  float dot = vx * wx + vy * wy;
  float t = dot / v_len2;
  inside_segment = (t >= 0.0f && t <= 1.0f);

  // Coordinates of projection point of P onto AB
  float proj_x = t * vx;
  float proj_y = t * vy;

  // Perpendicular vector components
  float perp_x = wx - proj_x;
  float perp_y = wy - proj_y;

  // Return its length
  return std::sqrt(perp_x * perp_x +
                   perp_y * perp_y);
}

}  // namespace linemapdraft_builder::geometry_utils