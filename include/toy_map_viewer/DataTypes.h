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