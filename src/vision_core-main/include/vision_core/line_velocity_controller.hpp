#pragma once

#include <cstddef>
#include <deque>
#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

class LineVelocityController {
public:
  LineVelocityController();
  explicit LineVelocityController(const RuleConfig &config,
                                  double observation_dt);

  void SetPath(const std::vector<Point2> &path_xy,
               const std::vector<double> &path_s,
               const std::vector<double> &path_heading);
  void ClearPath();
  void Observe(const Features &features, const Pose2 &pose);
  Command Compute(const Features &features);
  // 재탐색 판정은 유지하면서 현재 영상 특징으로 직접 추종한다.
  Command ComputeTracking(const Features &features);
  // 라인을 확정하기 전에는 전진하지 않고 기존 방향 기억으로 회전 탐색한다.
  Command ComputeSearchRotation();
  // 라인 재진입에서는 가까운 점과 기울기를 섞지 않고 먼 점만 추종한다.
  Command ComputeLookaheadApproach(const Features &features);
  bool InRecovery() const;
  // 미션 종료 후 라인 재탐색을 시작하되 기존 경로/방향 기억은 보존한다.
  void BeginReacquisition();
  void Reset();

private:
  std::size_t NearestPathIndex(double min_s, double max_s) const;
  Command MemoryRecoveryCommand() const;
  Command ComputeImpl(const Features &features, bool use_memory_recovery);

  RuleConfig config_;
  double observation_dt_{};
  double last_vx_{0.0};
  double last_wz_{0.0};
  Pose2 pose_{};
  std::vector<Point2> path_xy_;
  std::vector<double> path_s_;
  std::vector<double> path_heading_;
  bool path_memory_valid_{false};
  std::size_t path_memory_index_{0};
  double path_memory_s_{0.0};
  double path_anchor_s_{0.0};
  double path_anchor_x_{0.0};
  double path_anchor_y_{0.0};
  double line_side_memory_{0.0};
  std::size_t lost_observations_{0};
  std::deque<bool> reliable_history_;
  bool recovery_active_{true};
};

} // namespace vision_core
