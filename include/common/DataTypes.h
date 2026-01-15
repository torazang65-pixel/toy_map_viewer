#pragma once

#include <vector>
#include <cstdint>

struct Point6D
{
    double x, y, z;
    double dx, dy, dz;
};

struct Lane
{
    int id;
    int type;
    bool valid;
    bool explicit_lane;
    std::vector<Point6D> points;
};

struct VoxelPoint {
    float x, y, z, yaw;
    uint32_t density;
};