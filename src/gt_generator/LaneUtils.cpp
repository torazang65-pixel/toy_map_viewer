// LaneUtils.cpp
#include "gt_generator/LaneUtils.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <numeric>
#include <set>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

using linemapdraft_builder::data_types::Lane;
using linemapdraft_builder::data_types::Point6D;

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
    struct Edge {
        int u, v;
        double weight;
        bool operator<(const Edge& other) const {
            return weight < other.weight;
        }
    };

    struct UnionFind {
        std::vector<int> parent;
        UnionFind(int n) {
            parent.resize(n);
            std::iota(parent.begin(), parent.end(), 0);
        }
        int find(int x) {
            if (parent[x] == x) return x;
            return parent[x] = find(parent[x]);
        }
        bool unite(int x, int y) {
            int rootX = find(x);
            int rootY = find(y);
            if (rootX != rootY) {
                parent[rootX] = rootY;
                return true;
            }
            return false;
        }
    };

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

    void ReorderPointsImproved(Lane& lane)
    {
        if (lane.points.size() < 3) return;

        int N = lane.points.size();

        // 1. KD-Tree 구성
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.reserve(N);
        for (const auto& pt : lane.points) {
            cloud->points.emplace_back(pt.x, pt.y, pt.z);
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        // 2. K-NN Graph 생성 (Edge List)
        // K가 너무 작으면 끊어질 수 있고, 너무 크면 멀리 있는 점과 연결됨.
        // Voxel Grid가 0.5m이므로 K=5~10 정도면 충분히 로컬 연결성 확보 가능.
        int K = std::min(N - 1, 10); 
        std::vector<Edge> edges;
        edges.reserve(N * K);

        for (int i = 0; i < N; ++i) {
            std::vector<int> idxs;
            std::vector<float> sq_dists;
            kdtree.nearestKSearch(cloud->points[i], K + 1, idxs, sq_dists); // +1은 자기 자신 포함

            for (size_t j = 1; j < idxs.size(); ++j) {
                int neighbor = idxs[j];
                if (i < neighbor) { // 중복 방지 (무방향 그래프)
                    edges.push_back({i, neighbor, (double)sq_dists[j]}); // 거리 제곱 그대로 사용 (비교용)
                }
            }
        }
        
        // 거리 순 정렬 (Kruskal)
        std::sort(edges.begin(), edges.end());

        // 3. MST 구성 (Kruskal's Algorithm)
        UnionFind uf(N);
        std::vector<std::vector<int>> adj(N);
        int edges_count = 0;

        for (const auto& edge : edges) {
            if (uf.unite(edge.u, edge.v)) {
                adj[edge.u].push_back(edge.v);
                adj[edge.v].push_back(edge.u);
                edges_count++;
            }
        }

        // 4. 가장 큰 연결 요소(Connected Component) 찾기
        // (PCALaneGenerator가 끊어진 점들을 만들 수도 있으므로 가장 큰 덩어리만 살림)
        std::vector<bool> visited(N, false);
        int max_comp_start_node = -1;
        int max_comp_size = -1;

        for (int i = 0; i < N; ++i) {
            if (visited[i]) continue;
            
            std::vector<int> component;
            std::queue<int> q;
            q.push(i);
            visited[i] = true;
            component.push_back(i);

            while(!q.empty()){
                int u = q.front(); q.pop();
                for(int v : adj[u]){
                    if(!visited[v]){
                        visited[v] = true;
                        component.push_back(v);
                        q.push(v);
                    }
                }
            }

            if ((int)component.size() > max_comp_size) {
                max_comp_size = component.size();
                max_comp_start_node = i;
            }
        }

        if (max_comp_start_node == -1) return; // Should not happen

        // 5. 트리 지름(Diameter)의 양 끝점 찾기 (Two-pass BFS)
        // BFS Helper
        auto bfs_farthest = [&](int start_node) -> std::pair<int, std::vector<int>> {
            std::queue<int> q;
            q.push(start_node);
            
            std::vector<int> dist(N, -1);
            std::vector<int> parent(N, -1);
            dist[start_node] = 0;

            int farthest_node = start_node;
            int max_dist = 0;

            while (!q.empty()) {
                int u = q.front(); q.pop();
                if (dist[u] > max_dist) {
                    max_dist = dist[u];
                    farthest_node = u;
                }

                for (int v : adj[u]) {
                    if (dist[v] == -1) { // 방문 안함 (같은 컴포넌트 내에서만 이동됨)
                        dist[v] = dist[u] + 1;
                        parent[v] = u;
                        q.push(v);
                    }
                }
            }
            return {farthest_node, parent};
        };

        // Pass 1: 임의 점 -> 끝점 U
        auto [u, _] = bfs_farthest(max_comp_start_node);
        // Pass 2: 끝점 U -> 반대편 끝점 V (Parent 정보 저장)
        auto [v, parents] = bfs_farthest(u);

        // 6. 경로 역추적 (V -> ... -> U)
        std::vector<Point6D> ordered_points;
        int curr = v;
        while (curr != -1) {
            ordered_points.push_back(lane.points[curr]);
            curr = parents[curr];
        }

        // 7. [중요] 스무딩 (Smoothing) - MST 경로는 지그재그일 수 있음
        // 3-Point Moving Average를 적용하여 튀는 점 보정
        if (ordered_points.size() >= 3) {
            std::vector<Point6D> smoothed = ordered_points;
            for (size_t i = 1; i < ordered_points.size() - 1; ++i) {
                smoothed[i].x = (ordered_points[i-1].x + ordered_points[i].x + ordered_points[i+1].x) / 3.0;
                smoothed[i].y = (ordered_points[i-1].y + ordered_points[i].y + ordered_points[i+1].y) / 3.0;
                smoothed[i].z = (ordered_points[i-1].z + ordered_points[i].z + ordered_points[i+1].z) / 3.0;
            }
            ordered_points = smoothed;
        }

        // 8. 방향 벡터(dx, dy, dz) 재계산
        for (size_t i = 0; i < ordered_points.size(); ++i) {
            Point6D& p = ordered_points[i];
            double dx, dy, dz;

            if (i < ordered_points.size() - 1) {
                // Forward difference
                dx = ordered_points[i+1].x - p.x;
                dy = ordered_points[i+1].y - p.y;
                dz = ordered_points[i+1].z - p.z;
            } else {
                // Backward difference (last point)
                dx = p.x - ordered_points[i-1].x;
                dy = p.y - ordered_points[i-1].y;
                dz = p.z - ordered_points[i-1].z;
            }
            
            double len = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (len > 1e-6) {
                p.dx = dx / len; p.dy = dy / len; p.dz = dz / len;
            }
        }

        lane.points = ordered_points;
    }
}
