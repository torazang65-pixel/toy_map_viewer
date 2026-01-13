#pragma once

struct MapPoint {
    double x;
    double y;
    double z;
    double yaw;
    // double intensity // add if needed

    MapPoint(double _x=0, double _y=0, double _z=0, double _yaw=0) : x(_x), y(_y), z(_z), yaw(_yaw) {}
};

// 굳이 필요 없을지도?
// 어차피 각 점에 대한 정보는 PointCloud XYZI에서 처리 가능할 지 도?