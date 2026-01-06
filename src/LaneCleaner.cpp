#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <algorithm> // std::max, std::min
#include "toy_map_viewer/DataTypes.h"
#include "toy_map_viewer/LaneUtils.h" // CalculateLaneLength 사용


namespace LaneCleaner {

// KD-Tree 검색용 구조체
struct PointSourceInfo {
    int lane_id;
    // int point_idx; // For debugging if needed
};

void TrimOverlappingLaneEnds(std::map<int, Lane>& global_map, const LaneConfig& config) {
    const double OVERLAP_RADIUS = config.overlap_radius; // 30cm 이내
    const double LINEARITY_TOLERANCE = config.linearity_tolerance;

    // 1. 각 Lane의 길이 미리 계산 (비교 기준)
    std::map<int, double> lane_lengths;
    std::map<int, double> lane_linearities;

    for (const auto& pair : global_map) {
        lane_lengths[pair.first] = LaneUtils::CalculateLaneLength(pair.second);
        lane_linearities[pair.first] = LaneUtils::CalculateLaneLinearity(pair.second);
    }

    // 2. 전체 포인트 클라우드 빌드 (Look-up용)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<PointSourceInfo> cloud_info; 

    for (const auto& pair : global_map) {
        int l_id = pair.first;
        for (const auto& p : pair.second.points) {
            pcl::PointXYZ pt;
            pt.x = p.x; pt.y = p.y; pt.z = p.z;
            cloud->push_back(pt);
            cloud_info.push_back({l_id});
        }
    }

    if (cloud->empty()) return;

    // 3. KD-Tree 생성
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 삭제할 Lane ID들을 담을 리스트
    std::vector<int> lanes_to_remove;

    // 4. 모든 Lane을 순회하며 양 끝단 검사 (Trim Candidate)
    for (auto& pair : global_map) {
        int my_id = pair.first;
        Lane& my_lane = pair.second;
        double my_len = lane_lengths[my_id];
        
        int n_points = (int)my_lane.points.size();
        if (n_points < 2) continue;

        int trim_front_count = 0;
        int trim_back_count = 0;

        // --- [Front Trimming] 앞에서부터 안쪽으로 ---
        for (int i = 0; i < n_points; ++i) {
            pcl::PointXYZ query_pt;
            query_pt.x = my_lane.points[i].x;
            query_pt.y = my_lane.points[i].y;
            query_pt.z = my_lane.points[i].z;

            std::vector<int> idxs;
            std::vector<float> dists;
            bool should_remove = false;

            if (kdtree.radiusSearch(query_pt, OVERLAP_RADIUS, idxs, dists) > 0) {
                for (int neighbor_idx : idxs) {
                    int other_id = cloud_info[neighbor_idx].lane_id;
                    if (other_id == my_id) continue; // 내 점은 무시

                    double my_linearity = lane_linearities[my_id];
                    double other_linearity = lane_linearities[other_id];

                    if (other_linearity > my_linearity + LINEARITY_TOLERANCE) {
                        should_remove = true;
                    }
                    else if (std::abs(other_linearity - my_linearity) < LINEARITY_TOLERANCE) {
                        // 선형성이 비슷할 때 길이 비교
                        double my_len = lane_lengths[my_id];
                        double other_len = lane_lengths[other_id];

                        // 판단: 상대가 더 길거나, 길이가 같은데 상대 ID가 더 작으면 내가 양보(삭제)
                        // (ID 비교는 Tie-breaking: 일관된 결과를 위해)
                        if (other_len > my_len) {
                            should_remove = true; 
                        } else if (std::abs(other_len - my_len) < 1e-3 && other_id < my_id) {
                        should_remove = true;
                        }
                    }

                    if (should_remove) break;
                }
            }

            if (should_remove) {
                trim_front_count++;
            } else {
                // 겹치지 않는 점이 나오면, 즉시 중단 (중간은 자르지 않음)
                break;
            }
        }

        // 만약 앞에서 다 지워졌다면 뒤는 볼 필요 없음
        if (trim_front_count >= n_points) {
            lanes_to_remove.push_back(my_id);
            continue; 
        }

        // --- [Back Trimming] 뒤에서부터 안쪽으로 ---
        // (앞에서 지워질 예정인 인덱스 전까지만 검사)
        for (int i = n_points - 1; i >= trim_front_count; --i) {
            pcl::PointXYZ query_pt;
            query_pt.x = my_lane.points[i].x;
            query_pt.y = my_lane.points[i].y;
            query_pt.z = my_lane.points[i].z;

            std::vector<int> idxs;
            std::vector<float> dists;
            bool should_remove = false;

            if (kdtree.radiusSearch(query_pt, OVERLAP_RADIUS, idxs, dists) > 0) {
                for (int neighbor_idx : idxs) {
                    int other_id = cloud_info[neighbor_idx].lane_id;
                    if (other_id == my_id) continue; // 내 점은 무시

                    double my_linearity = lane_linearities[my_id];
                    double other_linearity = lane_linearities[other_id];

                    if (other_linearity > my_linearity + LINEARITY_TOLERANCE) {
                        should_remove = true;
                    }
                    else if (std::abs(other_linearity - my_linearity) < LINEARITY_TOLERANCE) {
                        // 선형성이 비슷할 때 길이 비교
                        double my_len = lane_lengths[my_id];
                        double other_len = lane_lengths[other_id];

                        // 판단: 상대가 더 길거나, 길이가 같은데 상대 ID가 더 작으면 내가 양보(삭제)
                        // (ID 비교는 Tie-breaking: 일관된 결과를 위해)
                        if (other_len > my_len) {
                            should_remove = true; 
                        } else if (std::abs(other_len - my_len) < 1e-3 && other_id < my_id) {
                        should_remove = true;
                        }
                    }

                    if (should_remove) break;
                }
            }

            if (should_remove) {
                trim_back_count++;
            } else {
                break; // 겹치지 않는 점 나오면 중단
            }
        }

        // 5. 실제 Trimming 적용
        if (trim_front_count > 0 || trim_back_count > 0) {
            // 전체가 다 잘려나가는 경우
            if (trim_front_count + trim_back_count >= n_points) {
                lanes_to_remove.push_back(my_id);
            } else {
                // Vector erase 사용
                // 뒤쪽 삭제
                if (trim_back_count > 0) {
                    my_lane.points.erase(my_lane.points.end() - trim_back_count, my_lane.points.end());
                }
                // 앞쪽 삭제
                if (trim_front_count > 0) {
                    my_lane.points.erase(my_lane.points.begin(), my_lane.points.begin() + trim_front_count);
                }
            }
        }
    }

    // 6. 빈 Lane 삭제
    for (int id : lanes_to_remove) {
        global_map.erase(id);
    }
}
} // namespace LaneCleaner