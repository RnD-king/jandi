// 처리 순서: [점선 분기 3단계] line_feature_extractor 뒤에 매 프레임 호출한다.
// 역할: Features와 로봇 자세/경로를 이용해 점선 추종 또는 점선 누락 복구 명령을 계산한다.
// 값 전달: Observe()가 특징/자세를 내부 상태에 기록하고, Compute()가 Command(vx, wz)를 반환한다.
// 다음 단계: 점선 후보 명령은 motion_command_selector.cpp의 최종 선택 입력으로 간다.

#include "vision_core/line_velocity_controller.hpp"

#include "vision_core/config_loader.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vision_core {
namespace {
double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
double Sign(double value) {
  return (value > 0.0) ? 1.0 : ((value < 0.0) ? -1.0 : 0.0);
}
double WrapAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}
const MissionControllerConfig &SharedAlgorithmConfig() {
  static const MissionControllerConfig config = LoadDefaultAlgorithmConfig();
  return config;
}
} // namespace

LineVelocityController::LineVelocityController()
    : LineVelocityController(SharedAlgorithmConfig().line,
                             SharedAlgorithmConfig().line_observation_dt) {}

LineVelocityController::LineVelocityController(const RuleConfig &config,
                                               double observation_dt)
    : config_(config), observation_dt_(std::max(observation_dt, 1e-6)) {}

void LineVelocityController::SetPath(
    const std::vector<Point2> &path_xy, const std::vector<double> &path_s,
    const std::vector<double> &path_heading) {
  if (path_xy.size() != path_s.size() || path_xy.size() != path_heading.size()) {
    ClearPath();
    return;
  }
  path_xy_ = path_xy;
  path_s_ = path_s;
  path_heading_ = path_heading;
  path_memory_valid_ = false;
  lost_observations_ = 0;
}

void LineVelocityController::ClearPath() {
  path_xy_.clear();
  path_s_.clear();
  path_heading_.clear();
  path_memory_valid_ = false;
  lost_observations_ = 0;
}

std::size_t LineVelocityController::NearestPathIndex(double min_s,
                                                     double max_s) const {
  if (path_xy_.empty()) return 0;
  const auto begin_it = std::lower_bound(path_s_.begin(), path_s_.end(), min_s);
  const auto end_it = std::upper_bound(path_s_.begin(), path_s_.end(), max_s);
  std::size_t begin = static_cast<std::size_t>(begin_it - path_s_.begin());
  std::size_t end = static_cast<std::size_t>(end_it - path_s_.begin());
  begin = std::min(begin, path_xy_.size() - 1);
  end = std::max(begin + 1, std::min(end, path_xy_.size()));
  std::size_t best = begin;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i < end; ++i) {
    const double dx = path_xy_[i].u - pose_.x;
    const double dy = path_xy_[i].v - pose_.y;
    const double distance = dx * dx + dy * dy;
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

void LineVelocityController::Observe(const Features &features,
                                     const Pose2 &pose) {
  pose_ = pose;
  const bool reliable = features.n_visible >= 3.0 &&
                        std::abs(features.u_err_near) <= 0.70;
  const bool reacquisition_hit =
      features.n_visible >= config_.line_reacquire_nvis &&
      std::abs(features.u_err_near) < config_.line_reacquire_u;
  reliable_history_.push_back(reacquisition_hit);
  while (static_cast<int>(reliable_history_.size()) >
         std::max(1, config_.line_stable_window)) {
    reliable_history_.pop_front();
  }
  const int reliable_hits = static_cast<int>(
      std::count(reliable_history_.begin(), reliable_history_.end(), true));
  recovery_active_ =
      reliable_hits < std::max(1, config_.line_stable_min_hits);
  if (path_xy_.empty()) {
    if (reliable) {
      line_side_memory_ =
          (1.0 - config_.recover_side_memory_alpha) * line_side_memory_ +
          config_.recover_side_memory_alpha * features.u_err_near;
      lost_observations_ = 0;
    } else {
      ++lost_observations_;
    }
    return;
  }
  if (!path_memory_valid_ && !reliable) {
    ++lost_observations_;
    return;
  }

  double min_s = 0.0;
  double max_s = path_s_.back();
  if (path_memory_valid_) {
    const double distance_from_anchor =
        std::hypot(pose_.x - path_anchor_x_, pose_.y - path_anchor_y_);
    min_s = std::max(0.0, path_memory_s_ - config_.recover_path_backtrack_m);
    max_s = std::min(
        path_s_.back(),
        std::max(path_memory_s_ + config_.recover_path_forward_margin_m,
                 path_anchor_s_ + distance_from_anchor +
                     config_.recover_path_forward_margin_m));
  }
  path_memory_index_ = NearestPathIndex(min_s, max_s);
  path_memory_s_ = path_s_[path_memory_index_];
  if (reliable) {
    path_memory_valid_ = true;
    path_anchor_s_ = path_memory_s_;
    path_anchor_x_ = pose_.x;
    path_anchor_y_ = pose_.y;
    line_side_memory_ =
        (1.0 - config_.recover_side_memory_alpha) * line_side_memory_ +
        config_.recover_side_memory_alpha * features.u_err_near;
    lost_observations_ = 0;
  } else {
    ++lost_observations_;
  }
}

Command LineVelocityController::MemoryRecoveryCommand() const {
  Command command{};
  const double lost_s = static_cast<double>(lost_observations_) * observation_dt_;
  if (!path_memory_valid_ || path_xy_.empty()) {
    double direction = -Sign(line_side_memory_);
    if (direction == 0.0) direction = Sign(last_wz_);
    if (direction == 0.0) direction = 1.0;
    command.vx = config_.recover_vx;
    command.wz = direction * config_.recover_wz;
    return command;
  }

  const double target_s =
      std::min(path_s_.back(), path_memory_s_ + config_.recover_lookahead_m);
  std::size_t target = static_cast<std::size_t>(
      std::lower_bound(path_s_.begin(), path_s_.end(), target_s) - path_s_.begin());
  target = std::min(target, path_xy_.size() - 1);
  const std::size_t current = std::min(path_memory_index_, path_xy_.size() - 1);
  const double bearing = std::atan2(path_xy_[target].v - pose_.y,
                                    path_xy_[target].u - pose_.x);
  const double bearing_error = WrapAngle(bearing - pose_.yaw);
  const double heading = path_heading_[current];
  const double heading_error = WrapAngle(heading - pose_.yaw);
  const double normal_x = -std::sin(heading);
  const double normal_y = std::cos(heading);
  const double cross_track = normal_x * (pose_.x - path_xy_[current].u) +
                             normal_y * (pose_.y - path_xy_[current].v);
  const double geometry_w = config_.recover_k_bearing * bearing_error +
                            config_.recover_k_heading * heading_error -
                            config_.recover_k_cross_track * cross_track;

  if (lost_s <= config_.recover_coast_s) {
    command.vx = 0.08;
    command.wz = Clamp(geometry_w, -config_.recover_search_wz_max,
                       config_.recover_search_wz_max);
    return command;
  }
  double preferred = Sign(geometry_w);
  if (preferred == 0.0) preferred = -Sign(line_side_memory_);
  if (preferred == 0.0) preferred = Sign(last_wz_);
  if (preferred == 0.0) preferred = 1.0;
  const double search_time =
      std::max(0.0, lost_s - config_.recover_search_delay_s);
  const int phase = static_cast<int>(
      search_time / std::max(config_.recover_sweep_period_s, 1e-6));
  const double direction = (phase % 3 < 2) ? preferred : -preferred;
  const double scale = std::min(config_.recover_search_wz_max,
                                config_.recover_search_wz_min + 0.08 * phase);
  command.vx = 0.0;
  command.wz = Clamp(geometry_w + direction * scale,
                     -config_.recover_search_wz_max,
                     config_.recover_search_wz_max);
  return command;
}

Command LineVelocityController::ComputeImpl(const Features &features,
                                            bool use_memory_recovery) {
  double w_nom = -config_.k_u * features.u_err_ctrl -
                 config_.k_slope * features.slope;
  double v_nom = config_.v_base - config_.k_v_u * std::abs(features.u_err_ctrl) -
                 config_.k_v_slope * std::abs(features.slope);
  if (features.n_visible < config_.low_visible_n) {
    w_nom = config_.low_visible_wz_decay * last_wz_;
    v_nom = config_.low_visible_vx;
  }
  v_nom *= std::max(0.0, config_.tracking_speed_scale);

  const bool recovery = use_memory_recovery && recovery_active_;
  Command raw = recovery ? MemoryRecoveryCommand() : Command{v_nom, w_nom};
  if (!path_memory_valid_ && features.n_visible < config_.no_visible_n) {
    raw.vx = config_.no_visible_vx;
    raw.wz = config_.no_visible_wz_decay * last_wz_;
  }
  const double vx_min = (recovery && path_memory_valid_) ? 0.0 : config_.cmd_vx_min;
  double vx = Clamp(raw.vx, vx_min, config_.cmd_vx_max);
  double wz = Clamp(raw.wz, config_.cmd_wz_min, config_.cmd_wz_max);
  vx = Clamp(vx, last_vx_ - config_.dv_max, last_vx_ + config_.dv_max);
  wz = Clamp(wz, last_wz_ - config_.dw_max, last_wz_ + config_.dw_max);
  last_vx_ = vx;
  last_wz_ = wz;
  return {vx, wz};
}

Command LineVelocityController::Compute(const Features &features) {
  // recovery 판정 중이어도 라인이 2점 이상 보이면 현재 영상의 강화된
  // lookahead를 따른다. 점이 부족할 때만 저장된 경로 기반 탐색을 사용한다.
  const bool use_memory_recovery = features.n_visible < config_.low_visible_n;
  return ComputeImpl(features, use_memory_recovery);
}

Command LineVelocityController::ComputeTracking(const Features &features) {
  return ComputeImpl(features, false);
}

Command LineVelocityController::ComputeSearchRotation() {
  Command raw = MemoryRecoveryCommand();
  raw.vx = 0.0;
  const double vx = 0.0;
  double wz = Clamp(raw.wz, config_.cmd_wz_min, config_.cmd_wz_max);
  wz = Clamp(wz, last_wz_ - config_.dw_max, last_wz_ + config_.dw_max);
  last_vx_ = vx;
  last_wz_ = wz;
  return {vx, wz};
}

Command LineVelocityController::ComputeLookaheadApproach(
    const Features &features) {
  const double error = features.u_err_lookahead;
  double vx = std::min(config_.line_recovery_vx_max,
                       config_.v_base - config_.k_v_u * std::abs(error));
  double wz = -config_.k_u * error;
  vx = Clamp(vx, config_.cmd_vx_min, config_.line_recovery_vx_max);
  wz = Clamp(wz, config_.cmd_wz_min, config_.cmd_wz_max);
  vx = Clamp(vx, last_vx_ - config_.dv_max, last_vx_ + config_.dv_max);
  wz = Clamp(wz, last_wz_ - config_.dw_max, last_wz_ + config_.dw_max);
  last_vx_ = vx;
  last_wz_ = wz;
  return {vx, wz};
}

bool LineVelocityController::InRecovery() const { return recovery_active_; }

void LineVelocityController::BeginReacquisition() {
  reliable_history_.clear();
  recovery_active_ = true;
}

void LineVelocityController::Reset() {
  last_vx_ = 0.0;
  last_wz_ = 0.0;
  pose_ = {};
  path_memory_valid_ = false;
  path_memory_index_ = 0;
  path_memory_s_ = 0.0;
  path_anchor_s_ = 0.0;
  path_anchor_x_ = 0.0;
  path_anchor_y_ = 0.0;
  line_side_memory_ = 0.0;
  lost_observations_ = 0;
  reliable_history_.clear();
  recovery_active_ = true;
}

} // namespace vision_core
