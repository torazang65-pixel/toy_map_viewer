// LaneUtils.cpp
#include "toy_map_viewer/LaneUtils.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ToDo: 현재 Point6D를 기준으로 작성되어 있음.
//       필요시 pcdPointXYZ 등으로 오버로딩 필요.
//       LaneMerger나 LaneCleaner 모두 pcd에서 kdtree를 사용하므로,
//       kdtree안에 들어가는 각 node는 pointXYZ 형태임.
//       따라서 Point6D안에 pointXYZ를 포함시키거나,
//       Point6D와 pointXYZ간 변환함수를 만들어야 할 수도 있음.

namespace 
{
    double GetDirectionScore(const Point6D& current, const Point6D& next) 
    {
        double vx = next.x - current.x;
        double vy = next.y - current.y;
        double vz = next.z - current.z;
        
        double dist = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (dist < 1e-6) return -1.0; 

        vx /= dist; vy /= dist; vz /= dist;

        return (current.dx * vx) + (current.dy * vy) + (current.dz * vz);
    }
}

namespace LaneUtils
{
    // Math Utils
    double GetDistanceSq(const Point6D& a, const Point6D& b) 
    {
        return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
    }
    double GetDistance(const Point6D& a, const Point6D& b) 
    {
        return std::sqrt(GetDistanceSq(a, b));
    }
    double GetAngleDegrees(const Point6D& v1, const Point6D& v2){
        double dot = v1.dx * v2.dx + v1.dy * v2.dy + v1.dz * v2.dz;
        double mag1 = std::sqrt(v1.dx * v1.dx + v1.dy * v1.dy + v1.dz * v1.dz);
        double mag2 = std::sqrt(v2.dx * v2.dx + v2.dy * v2.dy + v2.dz * v2.dz);
        if(mag1 < 1e-6 || mag2 < 1e-6) return 0.0;
        double cosAngle = dot / (mag1 * mag2);
        cosAngle = std::max(-1.0, std::min(1.0, cosAngle)); // Clamp
        return std::acos(cosAngle) * (180.0 / M_PI);
    }

    // Lane Utils
    void ReorderPoints(Lane& lane){
        
        if (lane.points.size() < 2) return;

        std::vector<Point6D> orderedPoints;
        std::vector<Point6D> remainingPoints = lane.points;
        
        // 1. 시작점 찾기 (가장 단순화된 버전: 앞쪽 방향으로 점들이 가장 많은 점)
        int startIdx = -1;

        int minBackwardCount = std::numeric_limits<int>::max();
        int maxForwardCount = -1;

        const double searchRadiusSq = 3.0 * 3.0; // 검색 반경 설정
        const double backwardThreshold = -0.3;  // 후방 판별 임계값

        for (size_t i = 0; i < remainingPoints.size(); ++i) {
            int backwardCount = 0;
            int forwardCount = 0;

            for (size_t j = 0; j < remainingPoints.size(); ++j) {
                if (i==j) continue;

                double distSq = GetDistanceSq(remainingPoints[i], remainingPoints[j]);
                if (distSq > searchRadiusSq) continue;

                double score = GetDirectionScore(remainingPoints[i], remainingPoints[j]);

                if (score < backwardThreshold) {
                    backwardCount++;
                } else if (score > 0.0) {
                    forwardCount++;
                }
            }

            if (backwardCount < minBackwardCount) {
                minBackwardCount = backwardCount;
                maxForwardCount = forwardCount;
                startIdx = i;
            } else if (backwardCount == minBackwardCount) {
                if (forwardCount > maxForwardCount) {
                    maxForwardCount = forwardCount;
                    startIdx = i;
                }
            }
        }

        /*
        double maxScore = -1.0;

        for (size_t i = 0; i < remainingPoints.size(); ++i) {
            int forwardCount = 0;
            for (size_t j = 0; j < remainingPoints.size(); ++j) {
                if (i == j) continue;
                if (GetDirectionScore(remainingPoints[i], remainingPoints[j]) > 0) {
                    forwardCount++;
                }
            }
            if (forwardCount > maxScore) {
                maxScore = forwardCount;
                startIdx = i;
            }
        }
        */

        if (startIdx == -1) startIdx = 0; // fallback

        orderedPoints.push_back(remainingPoints[startIdx]);
        remainingPoints.erase(remainingPoints.begin() + startIdx);

        // 2. Greedy Search
        while (!remainingPoints.empty()) {
            Point6D& current = orderedPoints.back();
            
            int bestIdx = -1;
            double minMetric = std::numeric_limits<double>::max(); 

            for (size_t i = 0; i < remainingPoints.size(); ++i) {
                double distSq = GetDistanceSq(current, remainingPoints[i]);
                double dirScore = GetDirectionScore(current, remainingPoints[i]);

                if (dirScore < -0.5) continue; // 역방향 제외 (필요시 조정)

                // 점수 계산: 거리가 가깝고 방향이 일치할수록 값이 작아짐
                double metric = distSq / (dirScore + 1.1); // +1.1은 0나눗셈 방지 및 가중치 조절

                if (metric < minMetric) {
                    minMetric = metric;
                    bestIdx = i;
                }
            }

            if (bestIdx != -1) {
                orderedPoints.push_back(remainingPoints[bestIdx]);
                remainingPoints.erase(remainingPoints.begin() + bestIdx);
            } else {
                // 연결 끊김 발생 시: 가장 가까운 점이라도 찾아서 잇기 (Fallback)
                double minDist = std::numeric_limits<double>::max();
                int closestIdx = -1;
                for(size_t i=0; i<remainingPoints.size(); ++i){
                     double d = GetDistanceSq(current, remainingPoints[i]);
                     if(d < minDist){
                         minDist = d;
                         closestIdx = i;
                     }
                }
                
                if(closestIdx != -1) {
                    orderedPoints.push_back(remainingPoints[closestIdx]);
                    remainingPoints.erase(remainingPoints.begin() + closestIdx);
                } else {
                    break; 
                }
            }
        }

        lane.points = orderedPoints;
    }
    double CalculateLaneLength(const Lane& lane)
    {
        if (lane.points.size() < 2) return 0.0;
        
        double total_length = 0.0;
        for (size_t i = 1; i < lane.points.size(); ++i) {
            double point_distance = GetDistance(lane.points[i-1], lane.points[i]);
            // 이상치 필터링
            if (point_distance > 10){
                std::cerr << "Warning: Ignoring abnormal point distance of " << point_distance << " between points " << i-1 << " and " << i << " at " << lane.id << std::endl;
                continue;
            }
            total_length += point_distance;
        }
        return total_length;
    }
    double CalculateLaneLinearity(const Lane& lane)
    {
        double total_length = CalculateLaneLength(lane);
        if (total_length < 1e-6) return 0.0;

        double direct_distance = GetDistance(lane.points.front(), lane.points.back());

        return direct_distance / total_length;
    }
}