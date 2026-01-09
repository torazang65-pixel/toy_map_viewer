#pragma once

#include <vector>

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

struct LidarPoint {
    float x;
    float y;
    float z;
    // float intensity; // 필요시 추가
    // char region[5];
    // int zone_idx;
};

struct LidarFrame {
    int id;
    std::vector<LidarPoint> points;
};
