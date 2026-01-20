#include <common/data_types.h>
#include <evaluator/polyline_evaluator.h>
#include <ros/ros.h>

#include <vector>

namespace linemapdraft_builder::polyline_evaluator {

// 알고리즘 정의
/*
1. gt에 대해 rtree를 빌드, matched 배열 초기화
2. preds에 대해 루프
- rtree를 이용해서 쿼리 thresh 만족하는 점들을 구함. -> 각도의 경우 무방향이라는걸 고려해야함. 마지막 점의 경우에는 yaw값을 예외적으로 이전 점을 향하게.
- 통과한 경우 pred와 대응하는 gt 점들에 모두 flag 설정
3. preds에 대해 루프
- flag가 설정된 연속된 점들을 하나의 클러스터로 만들어 pred_intersections에 넣어준다.
- flag가 설정되지 않은 연속된 점들을 하나의 클러스터로 만들어서 draft_fps에 넣어준다.
4. gt에 대해 루프
- flag가 설정된 연속된 점들을 하나의 클러스터로 만들어 gt_intersections에 넣어준다.
- flag가 설정되지 않은 연속된 점들을 하나의 클러스터로 만들어서 gt_fns에 넣어준다.
5. 지표 출력
- precision, recall, f1 score 계산
- TP = intersections의 길이 총합
- FP = draft_fps의 길이 총합
- FN = gt_fns의 길이 총합
- 이용해서 precision = TP / (TP + FP), recall = TP / (TP + FN), f1 = 2 * precision * recall / (precision + recall)로 계산
6. intersection, fp, fn을 polyline 바이너리로 출력, viewer로 확인할 수 있게
- draft_gt_intersections, draft_fps, gt_fns를 각각 io 호출 후 viewer에서 viz하도록 추가
*/

// 사실상 포인트 샘플링을 아주 짧게 가져가는 pointwise 알고리즘,
// Rtree를 이용하므로 포인트를 짧게 샘플링해도 되기 때문임

float get_yaw(const std::vector<std::vector<data_types::Point>> &polylines, size_t polyline_idx, size_t point_idx) {
  const std::vector<data_types::Point> &polyline = polylines[polyline_idx];
  if (polyline.size() == 1) return 0.0;

  const data_types::Point p1 = polyline[point_idx];
  const data_types::Point p2 = point_idx == polyline.size() - 1 ? polyline[point_idx - 1] : polyline[point_idx + 1];

  return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

void init_match_vec(const std::vector<std::vector<data_types::Point>> &preds,
                    const std::vector<std::vector<data_types::Point>> &gts,
                    std::vector<std::vector<bool>> &gt_matched,
                    std::vector<std::vector<bool>> &pred_matched) {
  gt_matched.resize(gts.size());
  for (size_t i = 0; i < gts.size(); ++i) {
    gt_matched[i].resize(gts[i].size(), false);
  }

  pred_matched.resize(preds.size());
  for (size_t i = 0; i < preds.size(); ++i) {
    pred_matched[i].resize(preds[i].size(), false);
  }
}

void insert_cluster_by_match(const std::vector<data_types::Point> &polyline,
                             const std::vector<bool> &matched,
                             std::vector<std::vector<data_types::Point>> &clusters_matched,
                             std::vector<std::vector<data_types::Point>> &clusters_unmatched) {
  std::vector<data_types::Point> cluster_buf;
  cluster_buf.reserve(polyline.size());

  bool matched_state = matched[0];
  for (size_t i = 0; i < polyline.size(); ++i) {
    if (matched[i] == matched_state) {
      cluster_buf.push_back(polyline[i]);
    } else {
      // matched가 반전될 때, sink로 flush 후 버퍼를 다시 채운다.
      auto &sink = matched_state ? clusters_matched : clusters_unmatched;
      sink.emplace_back(std::move(cluster_buf));

      cluster_buf.clear();
      cluster_buf.reserve(polyline.size() - i);
      cluster_buf.push_back(polyline[i]);
      matched_state = matched[i];
    }
  }

  auto &sink = matched_state ? clusters_matched : clusters_unmatched;
  sink.emplace_back(std::move(cluster_buf));
}

double polyline_length(const std::vector<data_types::Point> &polyline) {
  if (polyline.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < polyline.size(); ++i) {
    double dx = polyline[i].x - polyline[i - 1].x;
    double dy = polyline[i].y - polyline[i - 1].y;
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}

double total_length(const std::vector<std::vector<data_types::Point>> &polylines) {
  double sum = 0.0;
  for (const auto &polyline : polylines) {
    sum += polyline_length(polyline);
  }
  return sum;
}

// Z축 고려 X
void evaluate_len_coverage(const std::vector<std::vector<data_types::Point>> &preds,
                           const std::vector<std::vector<data_types::Point>> &gts,
                           std::vector<std::vector<data_types::Point>> &draft_intersections,
                           std::vector<std::vector<data_types::Point>> &gt_intersections,
                           std::vector<std::vector<data_types::Point>> &draft_fps,
                           std::vector<std::vector<data_types::Point>> &gt_fns,
                           float dist_th, float yaw_th) {
  // 1. gt에 대해 rtree를 빌드, matched 배열 초기화
  data_types::point_rtree rtree;
  build_rtree(rtree, gts);

  std::vector<std::vector<bool>> gt_matched;
  std::vector<std::vector<bool>> pred_matched;
  init_match_vec(preds, gts, gt_matched, pred_matched);

  // 2. preds에 대해 루프, rtree에서 thresh 만족하는 점들을 쿼리 & flagging
  const float cos_th = std::cos(yaw_th);
  for (size_t i = 0; i < preds.size(); ++i) {
    const auto &pred = preds[i];
    for (size_t j = 0; j < pred.size(); ++j) {
      float yaw1 = get_yaw(preds, i, j);
      data_types::bg_point p1(pred[j].x, pred[j].y);

      data_types::bg::model::box<data_types::bg_point> box(
          data_types::bg_point(pred[j].x - dist_th, pred[j].y - dist_th),
          data_types::bg_point(pred[j].x + dist_th, pred[j].y + dist_th));

      std::vector<data_types::point_value> query_result;

      rtree.query(data_types::bgi::intersects(box) && data_types::bgi::satisfies([&](const data_types::point_value &v) {
                    const auto &p2 = v.first;
                    float yaw2 = get_yaw(gts, v.second.polyline_idx, v.second.point_idx);
                    return data_types::bg::comparable_distance(p1, p2) < dist_th * dist_th && std::abs(std::cos(yaw1 - yaw2)) > cos_th;
                  }),
                  std::back_inserter(query_result));

      if (!query_result.empty()) {
        pred_matched[i][j] = true;

        for (const auto &[_, gt_index] : query_result) {
          gt_matched[gt_index.polyline_idx][gt_index.point_idx] = true;
        }
      }
    }
  }

  // 3. preds에 대해 루프,
  // flag가 설정된 연속된 점들을 하나의 클러스터로 만들어 pred_intersections에 넣어준다.
  // flag가 설정되지 않은 연속된 점들을 하나의 클러스터로 만들어서 draft_fps에 넣어준다.
  for (size_t i = 0; i < preds.size(); ++i) {
    const auto &pred = preds[i];
    const auto &matched = pred_matched[i];
    if (pred.empty()) continue;
    insert_cluster_by_match(pred, matched, draft_intersections, draft_fps);
  }

  // 4. gt에 대해 루프
  // flag가 설정된 연속된 점들을 하나의 클러스터로 만들어 gt_intersections에 넣어준다.
  // flag가 설정되지 않은 연속된 점들을 하나의 클러스터로 만들어서 gt_fns에 넣어준다.
  for (size_t i = 0; i < gts.size(); ++i) {
    const auto &gt = gts[i];
    const auto &matched = gt_matched[i];
    if (gt.empty()) continue;
    insert_cluster_by_match(gt, matched, gt_intersections, gt_fns);
  }

  // 5. 지표 출력
  // precision, recall, f1 score 계산
  // TP = intersections의 길이 총합
  // FP = draft_fps의 길이 총합
  // FN = gt_fns의 길이 총합
  // 이용해서 precision = TP / (TP + FP), recall = TP / (TP + FN), f1 = 2 * precision * recall / (precision + recall)로 계산

  double gt_total_length = total_length(gts);
  double TP_precision = total_length(draft_intersections);
  double TP_recall = total_length(gt_intersections);
  double FP = total_length(draft_fps);
  double FN = total_length(gt_fns);

  double precision = (TP_precision + FP > 0) ? TP_precision / (TP_precision + FP) : 0.0;
  double recall = (TP_recall + FN > 0) ? TP_recall / (TP_recall + FN) : 0.0;
  double f1 = (precision + recall > 0) ? (2 * precision * recall) / (precision + recall) : 0.0;

  std::cout << "\n===== Length Coverage Evaluation Results =====\n";
  std::cout << std::setw(12) << "Metric" << " | " << std::setw(12) << "Value" << "\n";
  std::cout << "----------------------------------------\n";
  std::cout << std::setw(12) << "TP_precision(len)" << " | " << std::setw(12) << TP_precision << "\n";
  std::cout << std::setw(12) << "TP_recall(len)" << " | " << std::setw(12) << TP_recall << "\n";
  std::cout << std::setw(12) << "FP(len)" << " | " << std::setw(12) << FP << "\n";
  std::cout << std::setw(12) << "FN(len)" << " | " << std::setw(12) << FN << "\n";
  std::cout << std::setw(12) << "GT(len)" << " | " << std::setw(12) << gt_total_length << "\n";
  std::cout << std::setw(12) << "Precision" << " | " << std::setw(12)
            << std::fixed << std::setprecision(3) << precision << "\n";
  std::cout << std::setw(12) << "Recall" << " | " << std::setw(12) << recall << "\n";
  std::cout << std::setw(12) << "F1-score" << " | " << std::setw(12) << f1 << "\n";
  std::cout << "----------------------------------------\n";
  std::cout << precision * 100 << "% of draft line length is valid!!" << std::endl;
  std::cout << recall * 100 << "% of gt line length is generated!!" << std::endl;
  std::cout << "========================================\n\n";
}
}  // namespace linemapdraft_builder::polyline_evaluator