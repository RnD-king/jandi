// 처리 순서: 공 집기 완료 뒤 거리 검증된 백보드로 접근하고, 백보드 RGB-D 자세로 미세정렬한 뒤 라인에 복귀한다.
// 실제 슛 모션은 외부 모션 패키지의 역할이며, 현재는 정렬 뒤 2초 정지로 대체한다.

#include "vision_core/goal_controller.hpp"

#include "vision_core/config_loader.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
constexpr double kTimeEpsilon = 1e-9;
constexpr double kGeometryEpsilon = 1e-6;
double SafeDenominator(double value) { return std::max(value, 1e-6); }
bool IsZeroMotion(const MotionCommand &command) {
  return std::abs(command.vx) <= kTimeEpsilon &&
         std::abs(command.vy) <= kTimeEpsilon &&
         std::abs(command.wz) <= kTimeEpsilon;
}
double WrapAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}
} // namespace

GoalController::GoalController()
    : GoalController(LoadDefaultAlgorithmConfig().goal) {}

GoalPoseObservation EstimateGoalPoseFromEdgeDepths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, const Intrinsics &intrinsics, double confidence) {
  GoalPoseObservation observation;
  if (!std::isfinite(left_u_px) || !std::isfinite(right_u_px) ||
      !std::isfinite(left_depth_m) || !std::isfinite(right_depth_m) ||
      !std::isfinite(intrinsics.fx) ||
      std::abs(intrinsics.fx) <= kGeometryEpsilon ||
      left_depth_m <= 0.0 || right_depth_m <= 0.0) {
    return observation;
  }

  const double left_x =
      (left_u_px - intrinsics.cx) * left_depth_m / intrinsics.fx;
  const double right_x =
      (right_u_px - intrinsics.cx) * right_depth_m / intrinsics.fx;
  const double width_x = right_x - left_x;
  if (!std::isfinite(width_x) || width_x <= kGeometryEpsilon) {
    return observation;
  }

  observation.valid = true;
  observation.x_m = 0.5 * (left_x + right_x);
  observation.z_m = 0.5 * (left_depth_m + right_depth_m);
  observation.yaw_rad =
      std::atan2(left_depth_m - right_depth_m, width_x);
  observation.confidence = confidence;
  return observation;
}

GoalPoseObservation EstimateGoalPoseFromBackboardDepths(
    double center_u_px, double center_depth_m, double left_u_px,
    double left_depth_m, double right_u_px, double right_depth_m,
    const Intrinsics &intrinsics, double confidence) {
  GoalPoseObservation observation = EstimateGoalPoseFromEdgeDepths(
      left_u_px, left_depth_m, right_u_px, right_depth_m, intrinsics,
      confidence);
  if (!observation.valid || !std::isfinite(center_u_px) ||
      !std::isfinite(center_depth_m) || center_depth_m <= 0.0 ||
      !std::isfinite(intrinsics.fx) ||
      std::abs(intrinsics.fx) <= kGeometryEpsilon) {
    return {};
  }
  observation.x_m =
      (center_u_px - intrinsics.cx) * center_depth_m / intrinsics.fx;
  observation.z_m = center_depth_m;
  return observation;
}

GoalController::GoalController(const GoalConfig &config) : config_(config) {}

const char *GoalController::ModeName(GoalMode mode) {
  switch (mode) {
  case GoalMode::kLineFollow: return "LINE_FOLLOW";
  case GoalMode::kPostPickupWait: return "GOAL_POST_PICKUP_WAIT";
  case GoalMode::kTiltCameraToGoal: return "CAMERA_TILT_TO_GOAL_VIEW";
  case GoalMode::kSearch: return "GOAL_SEARCH";
  case GoalMode::kApproach: return "GOAL_APPROACH";
  case GoalMode::kFineAdjust: return "GOAL_FINE_ADJUST";
  case GoalMode::kShoot: return "GOAL_SHOOT";
  case GoalMode::kReturnCameraToLine: return "CAMERA_RETURN_TO_LINE_VIEW";
  case GoalMode::kHeadingRecovery: return "GOAL_HEADING_RECOVERY";
  case GoalMode::kRlStopping: return "GOAL_RL_STOPPING";
  }
  return "UNKNOWN";
}

double GoalController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

void GoalController::StartAfterPickup(double now_sec) {
  (void)now_sec;
  SetHasBall(true);
  goal_entry_armed_ = true;
}

void GoalController::SetHasBall(bool has_ball) {
  if (!has_ball) {
    has_ball_ = false;
    goal_entry_armed_ = false;
    post_pickup_line_wait_active_ = false;
    post_pickup_line_wait_start_sec_ = 0.0;
    if (mode_ == GoalMode::kLineFollow) ball_consumed_ = false;
    return;
  }
  if (!ball_consumed_ && !has_ball_) {
    has_ball_ = true;
    // 공을 들기 전에 보였던 골대 이력으로 즉시 진입하지 않고, 보유 상태가
    // 켜진 뒤의 프레임만 안정 검출 조건에 사용한다.
    ClearTracking();
    post_pickup_line_wait_active_ = false;
    post_pickup_line_wait_start_sec_ = 0.0;
  }
}

void GoalController::UpdateBallState(const BallResult &ball_result) {
  SetHasBall(ball_result.has_ball);
  const bool should_arm = has_ball_ && !ball_consumed_ &&
                          ball_result.mode == BallMode::kLineFollow;
  if (should_arm && !goal_entry_armed_) {
    // Ball mode가 완전히 끝나기 전에 보인 골대 프레임은 진입 안정화 이력에서
    // 제외한다.
    ClearTracking();
  }
  goal_entry_armed_ = should_arm;
  if (!should_arm) {
    post_pickup_line_wait_active_ = false;
    post_pickup_line_wait_start_sec_ = 0.0;
  }
}

GoalResult GoalController::Compute(
    const std::optional<ObjectTarget> &tracking_target, int image_width,
    int image_height, double now_sec, bool line_reference_valid,
    const CameraFeedback &camera_feedback) {
  return Compute(std::nullopt, tracking_target, GoalPoseObservation{},
                 image_width, image_height, now_sec, line_reference_valid,
                 camera_feedback);
}

GoalResult GoalController::Compute(
    const std::optional<ObjectTarget> &goal_target,
    const std::optional<ObjectTarget> &backboard_target,
    const GoalPoseObservation &goal_pose, int image_width, int image_height,
    double now_sec, bool line_reference_valid,
    const CameraFeedback &camera_feedback) {
  return Compute(goal_target, backboard_target, goal_pose, image_width,
                 image_height, now_sec, line_reference_valid, camera_feedback,
                 ActionExecutionFeedback{});
}

GoalResult GoalController::Compute(
    const std::optional<ObjectTarget> &goal_target,
    const std::optional<ObjectTarget> &backboard_target,
    const GoalPoseObservation &goal_pose, int image_width, int image_height,
    double now_sec, bool line_reference_valid,
    const CameraFeedback &camera_feedback,
    const ActionExecutionFeedback &action_feedback) {
  // 5-class 모델의 goal 출력은 호환을 위해 입력으로 유지하지만 골대 미션의
  // 진입·검색·접근은 거리 검증을 통과한 backboard만 사용한다.
  (void)goal_target;
  UpdateGoalTracker(backboard_target, image_width, image_height);
  UpdatePoseTracker(backboard_target, goal_pose);
  // 공 미션이 완전히 끝나고 라인을 다시 잡은 뒤에는 백보드가 아직 보이지
  // 않아도 LINE_FOLLOW를 유지한 채 시간을 잰다. 라인을 다시 잃으면 타이머를
  // 처음부터 다시 시작해 고개를 든 채 잘못된 방향으로 진입하지 않게 한다.
  if (mode_ == GoalMode::kLineFollow && has_ball_ && goal_entry_armed_) {
    if (!line_reference_valid) {
      post_pickup_line_wait_active_ = false;
      post_pickup_line_wait_start_sec_ = 0.0;
    } else {
      if (!post_pickup_line_wait_active_) {
        post_pickup_line_wait_active_ = true;
        post_pickup_line_wait_start_sec_ = now_sec;
      }
      if (now_sec - post_pickup_line_wait_start_sec_ + kTimeEpsilon >=
          std::max(0.0, config_.post_pickup_wait_sec)) {
        post_pickup_line_wait_active_ = false;
        mode_ = GoalMode::kTiltCameraToGoal;
        state_enter_sec_ = now_sec;
      }
    }
  }
  GoalResult result;
  result.mode = mode_;
  result.tracked = tracked_;
  result.pose = tracked_pose_;
  // 디버그 화면에서도 미세조정 중 예상 회전각을 확인할 수 있게 안정된
  // pose가 있으면 실시간 계산값을 제공한다. SHOOT 진입 시에는 아래에서
  // 같은 값을 latch한다.
  result.shoot_yaw_rad =
      tracked_pose_.visible ? ComputeShootYawRad() : shoot_yaw_rad_;

  switch (mode_) {
  case GoalMode::kLineFollow:
    return result;
  case GoalMode::kPostPickupWait:
    result.active = true;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.post_pickup_wait_sec)) {
      mode_ = GoalMode::kTiltCameraToGoal;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kGoal;
    }
    return result;
  case GoalMode::kTiltCameraToGoal:
    result.active = true;
    result.camera_request = CameraRequest::kGoal;
    // 라인 위 직선 정렬이 확인된 뒤 진입하는 상태이므로, 카메라가 골대
    // 시야에 도달할 때까지 yaw를 섞지 않고 그대로 전진한다.
    result.command = {std::max(0.0, config_.camera_tilt_forward_vx), 0.0, 0.0};
    if (camera_feedback.actual_mode == CameraMode::kGoal &&
        camera_feedback.settled) {
      mode_ = GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.command = {};
      result.camera_request = CameraRequest::kNone;
      result.tracked = {};
      result.pose = {};
    } else if (now_sec - state_enter_sec_ + kTimeEpsilon >=
               std::max(0.0, config_.camera_motion_timeout_sec)) {
      result.command = {};
    }
    return result;
  case GoalMode::kSearch:
    result.active = true;
    // 카메라를 골대 시야로 올린 뒤에는 제자리회전을 새로 만들지 않고,
    // 현재 자세에서 거리 검증된 백보드의 안정 검출을 기다린다.
    result.command = {};
    if (PoseReadyForFineAdjust()) {
      mode_ = action_feedback.enabled ? GoalMode::kRlStopping
                                      : GoalMode::kFineAdjust;
      state_enter_sec_ = now_sec;
      fine_adjust_history_.clear();
      ResetFineMotion();
      result.mode = mode_;
      if (action_feedback.enabled) {
        result.command = {};
      } else {
        result.action_request = GoalActionRequest::kFineAdjust;
        result.command = ComputeFineAdjustCommand(now_sec);
      }
    } else if (tracked_.stable && tracked_.visible) {
      mode_ = GoalMode::kApproach;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.command = ComputeApproachCommand();
    }
    return result;
  case GoalMode::kApproach: {
    result.active = true;
    if (tracked_.visible) result.command = ComputeApproachCommand();
    if (PoseReadyForFineAdjust()) {
      mode_ = action_feedback.enabled ? GoalMode::kRlStopping
                                      : GoalMode::kFineAdjust;
      state_enter_sec_ = now_sec;
      fine_adjust_history_.clear();
      ResetFineMotion();
      result.mode = mode_;
      if (action_feedback.enabled) {
        result.command = {};
      } else {
        result.action_request = GoalActionRequest::kFineAdjust;
        result.command = ComputeFineAdjustCommand(now_sec);
      }
      return result;
    }
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.command = {};
      return result;
    }
    return result;
  }
  case GoalMode::kRlStopping:
    result.active = true;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.rl_stop_duration_sec)) {
      mode_ = GoalMode::kFineAdjust;
      state_enter_sec_ = now_sec;
      fine_adjust_history_.clear();
      ResetFineMotion();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kFineAdjust;
      result.command = ComputeFineAdjustCommand(now_sec);
      if (action_feedback.enabled && IsZeroMotion(result.command)) {
        // 이미 정렬 허용범위에 들어온 경우에도 RL VELOCITY(0,0,0)으로
        // 돌아가지 않고, 정지자세 ACTION에서 연속 관측을 확인한다.
        fine_adjust_hold_active_ = true;
        result.action_request = GoalActionRequest::kFineAdjustHold;
      }
    }
    return result;
  case GoalMode::kFineAdjust: {
    result.active = true;
    if (action_feedback.enabled) {
      if (fine_adjust_hold_active_) {
        // STEP/TURN 뒤의 재측정 구간도 별도 ACTION이다. 이 ACTION이 실행되는
        // 동안 RL 속도 제어는 켜지지 않으며 DONE 뒤에만 자세를 다시 판단한다.
        result.action_request = GoalActionRequest::kFineAdjustHold;
        result.command = {};
        if (action_feedback.action_active && !action_feedback.action_done) {
          return result;
        }
        if (!action_feedback.action_done) return result;
        fine_adjust_hold_active_ = false;
        ResetFineMotion();
        result.action_request = GoalActionRequest::kFineAdjust;
      } else {
        result.action_request = GoalActionRequest::kFineAdjust;
        // 동일 미세걸음 action_id가 실행 중인 동안에는 새 관측으로 상태나
        // 동작 방향을 바꾸지 않는다. DONE 뒤에는 HOLD ACTION부터 실행한다.
        if (action_feedback.action_active && !action_feedback.action_done) {
          result.command = {};
          return result;
        }
        if (action_feedback.action_done) {
          ResetFineMotion();
          fine_adjust_hold_active_ = true;
          result.action_request = GoalActionRequest::kFineAdjustHold;
          result.command = {};
          return result;
        }
      }
    } else {
      result.action_request = GoalActionRequest::kFineAdjust;
    }
    if (pose_lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = tracked_.visible ? GoalMode::kApproach : GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      fine_adjust_history_.clear();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kNone;
      ResetFineMotion();
      result.command = tracked_.visible
                           ? ComputeApproachCommand()
                           : MotionCommand{};
      return result;
    }
    const int aligned_hits = static_cast<int>(std::count(
        fine_adjust_history_.begin(), fine_adjust_history_.end(), true));
    if (aligned_hits >= std::max(1, config_.fine_adjust_min_hits) &&
        FineAdjustSettled(now_sec)) {
      mode_ = GoalMode::kShoot;
      state_enter_sec_ = now_sec;
      shoot_yaw_rad_ = ComputeShootYawRad();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kShoot;
      result.shoot_yaw_rad = shoot_yaw_rad_;
      result.command = {};
      ResetFineMotion();
      return result;
    }
    if (tracked_pose_.visible) {
      result.command = ComputeFineAdjustCommand(now_sec);
    }
    if (action_feedback.enabled && IsZeroMotion(result.command)) {
      fine_adjust_hold_active_ = true;
      result.action_request = GoalActionRequest::kFineAdjustHold;
    }
    return result;
  }
  case GoalMode::kShoot:
    result.active = true;
    result.action_request = GoalActionRequest::kShoot;
    result.shoot_yaw_rad = shoot_yaw_rad_;
    result.command = {};
    if ((action_feedback.enabled && action_feedback.action_done) ||
        (!action_feedback.enabled &&
         now_sec - state_enter_sec_ + kTimeEpsilon >=
             std::max(0.0, config_.shoot_placeholder_sec))) {
      has_ball_ = false;
      ball_consumed_ = true;
      goal_entry_armed_ = false;
      post_pickup_line_wait_active_ = false;
      post_pickup_line_wait_start_sec_ = 0.0;
      mode_ = GoalMode::kReturnCameraToLine;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kNone;
      result.camera_request = CameraRequest::kForward;
      result.tracked = {};
      result.pose = {};
    }
    return result;
  case GoalMode::kReturnCameraToLine:
    result.active = true;
    result.camera_request = CameraRequest::kForward;
    result.command = {};
    if (camera_feedback.actual_mode == CameraMode::kForward &&
        camera_feedback.settled) {
      mode_ = GoalMode::kHeadingRecovery;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.active = false;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case GoalMode::kHeadingRecovery:
    // 이 상태에서는 line controller의 RECOV 명령을 그대로 사용한다.
    result.active = false;
    result.command = {};
    if (line_reference_valid) {
      mode_ = GoalMode::kLineFollow;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result = {};
      result.mode = GoalMode::kLineFollow;
    }
    return result;
  }
  return result;
}

void GoalController::UpdateGoalTracker(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height) {
  const bool detected = target.has_value() && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    hit_history_.pop_front();
  }

  if (detected) {
    lost_count_ = 0;
    TrackedGoal observed;
    observed.visible = true;
    const Point2 center = target->center_rectified
                              ? target->rectified_center_px
                              : target->center_px;
    observed.u_norm = Clamp(center.u / SafeDenominator(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(center.v / SafeDenominator(image_height), 0.0, 1.0);
    observed.h_norm = Clamp(
        target->height_px / SafeDenominator(image_height), 0.0, 1.0);
    observed.confidence = target->confidence;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      tracked_ = observed;
      has_smoothed_ = true;
    } else {
      tracked_.visible = true;
      tracked_.u_norm = (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm = (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = (1.0 - alpha) * tracked_.h_norm + alpha * observed.h_norm;
      tracked_.confidence = observed.confidence;
    }
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }

  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
}

void GoalController::UpdatePoseTracker(
    const std::optional<ObjectTarget> &backboard_target,
    const GoalPoseObservation &goal_pose) {
  const bool detected = backboard_target.has_value() && goal_pose.valid &&
                        std::isfinite(goal_pose.x_m) &&
                        std::isfinite(goal_pose.z_m) && goal_pose.z_m > 0.0 &&
                        std::isfinite(goal_pose.yaw_rad);
  pose_hit_history_.push_back(detected);
  while (static_cast<int>(pose_hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    pose_hit_history_.pop_front();
  }

  if (detected) {
    pose_lost_count_ = 0;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_pose_smoothed_) {
      tracked_pose_.x_m = goal_pose.x_m;
      tracked_pose_.z_m = goal_pose.z_m;
      tracked_pose_.yaw_rad = goal_pose.yaw_rad;
      has_pose_smoothed_ = true;
    } else {
      tracked_pose_.x_m =
          (1.0 - alpha) * tracked_pose_.x_m + alpha * goal_pose.x_m;
      tracked_pose_.z_m =
          (1.0 - alpha) * tracked_pose_.z_m + alpha * goal_pose.z_m;
      tracked_pose_.yaw_rad = WrapAngle(
          tracked_pose_.yaw_rad +
          alpha * WrapAngle(goal_pose.yaw_rad - tracked_pose_.yaw_rad));
    }
    tracked_pose_.visible = true;
    tracked_pose_.confidence = goal_pose.confidence;
  } else {
    ++pose_lost_count_;
    tracked_pose_.visible = false;
  }

  const int pose_hits = static_cast<int>(std::count(
      pose_hit_history_.begin(), pose_hit_history_.end(), true));
  tracked_pose_.stable =
      pose_hits >= std::max(1, config_.stable_min_hits);

  fine_enter_history_.push_back(
      detected && goal_pose.z_m <= config_.fine_adjust_start_z_m);
  while (static_cast<int>(fine_enter_history_.size()) >
         std::max(1, config_.stable_window)) {
    fine_enter_history_.pop_front();
  }

  fine_adjust_history_.push_back(detected && PoseAligned());
  while (static_cast<int>(fine_adjust_history_.size()) >
         std::max(1, config_.fine_adjust_window)) {
    fine_adjust_history_.pop_front();
  }
}

MotionCommand GoalController::ComputeApproachCommand() const {
  const double u_error = tracked_.u_norm - config_.target_u_norm;
  MotionCommand command;
  command.vx = std::max(0.0, config_.approach_vx);
  command.wz = Clamp(-config_.approach_wz_gain * u_error,
                     -std::abs(config_.approach_wz_max),
                     std::abs(config_.approach_wz_max));
  return command;
}

MotionCommand GoalController::MakeFineAdjustPulse() const {
  MotionCommand command;
  const double rim_x =
      tracked_pose_.x_m + config_.hoop_radius_m *
                                std::sin(tracked_pose_.yaw_rad);
  const double rim_z =
      tracked_pose_.z_m - config_.hoop_radius_m *
                                std::cos(tracked_pose_.yaw_rad);
  const double distance_error =
      std::hypot(rim_x, rim_z) - config_.throwing_range_m;

  // 림을 완전히 정면에 두는 횡이동/회전 대신 투척 가능 거리만 맞춘다.
  // 최종 림 방향 회전은 SHOOT action 실행기가 shoot_yaw_rad로 수행한다.
  command.vx = Clamp(config_.fine_vx_gain * distance_error,
                     -std::abs(config_.fine_vx_max),
                     std::abs(config_.fine_vx_max));
  if (std::abs(distance_error) > std::abs(config_.position_tolerance_m) &&
      std::abs(command.vx) < std::abs(config_.fine_translation_min)) {
    command.vx = std::copysign(std::abs(config_.fine_translation_min),
                               distance_error);
  }
  return command;
}

MotionCommand GoalController::ComputeFineAdjustCommand(double now_sec) {
  if (fine_motion_phase_ == FineMotionPhase::kPulse) {
    if (now_sec - fine_motion_phase_enter_sec_ + kTimeEpsilon <
        fine_pulse_duration_sec_) {
      // 움직이는 동안 우연히 허용범위를 통과한 관측은 완료 판정에 쓰지 않는다.
      fine_adjust_history_.clear();
      return fine_pulse_command_;
    }
    fine_motion_phase_ = FineMotionPhase::kSettle;
    fine_motion_phase_enter_sec_ = now_sec;
    fine_pulse_command_ = {};
    fine_adjust_history_.clear();
    return {};
  }

  if (fine_motion_phase_ == FineMotionPhase::kSettle) {
    if (now_sec - fine_motion_phase_enter_sec_ + kTimeEpsilon <
        std::max(0.0, config_.fine_settle_duration_sec)) {
      return {};
    }
    fine_motion_phase_ = FineMotionPhase::kReady;
  }

  if (PoseAligned()) return {};

  fine_pulse_command_ = MakeFineAdjustPulse();
  const bool translation = std::abs(fine_pulse_command_.vx) > 0.0 ||
                           std::abs(fine_pulse_command_.vy) > 0.0;
  const double rim_x =
      tracked_pose_.x_m + config_.hoop_radius_m * std::sin(tracked_pose_.yaw_rad);
  const double rim_z = tracked_pose_.z_m -
                       config_.hoop_radius_m * std::cos(tracked_pose_.yaw_rad);
  const double distance_error =
      std::hypot(rim_x, rim_z) - config_.throwing_range_m;
  const bool near_target = translation &&
      std::abs(distance_error) <= std::abs(config_.fine_near_error_m);
  fine_pulse_duration_sec_ = std::max(
      0.0, near_target ? config_.fine_near_pulse_duration_sec
                       : config_.fine_pulse_duration_sec);
  fine_motion_phase_ = FineMotionPhase::kPulse;
  fine_motion_phase_enter_sec_ = now_sec;
  fine_adjust_history_.clear();
  return fine_pulse_command_;
}

bool GoalController::FineAdjustSettled(double now_sec) const {
  if (fine_motion_phase_ == FineMotionPhase::kReady) return true;
  if (fine_motion_phase_ != FineMotionPhase::kSettle) return false;
  return now_sec - fine_motion_phase_enter_sec_ + kTimeEpsilon >=
         std::max(0.0, config_.fine_settle_duration_sec);
}

void GoalController::ResetFineMotion() {
  fine_motion_phase_ = FineMotionPhase::kReady;
  fine_pulse_command_ = {};
  fine_motion_phase_enter_sec_ = 0.0;
  fine_pulse_duration_sec_ = 0.0;
  fine_adjust_hold_active_ = false;
}

bool GoalController::PoseReadyForFineAdjust() const {
  const int hits = static_cast<int>(std::count(
      fine_enter_history_.begin(), fine_enter_history_.end(), true));
  return tracked_pose_.stable && tracked_pose_.visible &&
         hits >= std::max(1, config_.stable_min_hits);
}

bool GoalController::PoseAligned() const {
  if (!tracked_pose_.visible) return false;
  const double rim_x =
      tracked_pose_.x_m + config_.hoop_radius_m *
                                std::sin(tracked_pose_.yaw_rad);
  const double rim_z =
      tracked_pose_.z_m - config_.hoop_radius_m *
                                std::cos(tracked_pose_.yaw_rad);
  return std::abs(std::hypot(rim_x, rim_z) - config_.throwing_range_m) <=
         std::abs(config_.position_tolerance_m);
}

double GoalController::ComputeShootYawRad() const {
  const double rim_x =
      tracked_pose_.x_m + config_.hoop_radius_m *
                                std::sin(tracked_pose_.yaw_rad);
  const double rim_z =
      tracked_pose_.z_m - config_.hoop_radius_m *
                                std::cos(tracked_pose_.yaw_rad);
  if (!std::isfinite(rim_x) || !std::isfinite(rim_z) ||
      std::hypot(rim_x, rim_z) <= kGeometryEpsilon) {
    return 0.0;
  }
  // 카메라 x는 오른쪽(+), 로봇 yaw는 좌회전(+)이므로 부호를 반대로 한다.
  return WrapAngle(-std::atan2(rim_x, rim_z));
}

void GoalController::ClearTracking() {
  hit_history_.clear();
  pose_hit_history_.clear();
  fine_enter_history_.clear();
  fine_adjust_history_.clear();
  lost_count_ = 0;
  pose_lost_count_ = 0;
  has_smoothed_ = false;
  has_pose_smoothed_ = false;
  tracked_ = {};
  tracked_pose_ = {};
}

void GoalController::Reset() {
  mode_ = GoalMode::kLineFollow;
  state_enter_sec_ = 0.0;
  has_ball_ = false;
  ball_consumed_ = false;
  goal_entry_armed_ = false;
  post_pickup_line_wait_active_ = false;
  post_pickup_line_wait_start_sec_ = 0.0;
  shoot_yaw_rad_ = 0.0;
  ResetFineMotion();
  ClearTracking();
}

} // namespace vision_core

/*
ROS 통합 명령표 (ActionExecutionFeedback.enabled=true)
형식: MODE -> command_type, mission, mission_phase, velocity, action, camera

LINE_FOLLOW(0)
  -> VELOCITY, LINE(1), 0, line controller의 vx/vy/wz, NONE, NONE
GOAL_POST_PICKUP_WAIT(1)
  -> 외부 mode 숫자 호환용. 현재 새 실행에서는 LINE_FOLLOW 내부 타이머를 쓴다.
CAMERA_TILT_TO_GOAL_VIEW(2)
  -> VELOCITY, GOAL(3), 2, camera_tilt_forward_vx/0/0, NONE, GOAL(3)
GOAL_SEARCH(3)
  -> VELOCITY, GOAL(3), 3, 0/0/0, NONE, NONE
GOAL_APPROACH(4)
  -> VELOCITY, GOAL(3), 4, ComputeApproachCommand(), NONE, NONE
GOAL_RL_STOPPING(9)
  -> VELOCITY, GOAL(3), 9, 0/0/0, NONE, NONE
GOAL_FINE_ADJUST(5)
  -> 미세조정 pulse가 0이 아니면 ACTION, GOAL(3), 5, 0/0/0,
     STEP_FORWARD_HALF(3)/STEP_BACK(4)/LEFT_SIDE_STEP(5)/RIGHT_SIDE_STEP(6)/
     TURN_LEFT(7)/TURN_RIGHT(8) 중 오차축에 맞는 하나, NONE
  -> 정착·재측정 구간이면 ACTION, GOAL(3), 5, 0/0/0,
     DEFAULT_POSE_MODE(2), NONE
GOAL_SHOOT(6)
  -> ACTION, GOAL(3), 6, 0/0/0, SHOOT(17), NONE
CAMERA_RETURN_TO_LINE_VIEW(7)
  -> VELOCITY, GOAL(3), 7, 0/0/0, NONE, FORWARD(2)
GOAL_HEADING_RECOVERY(8)
  -> VELOCITY, GOAL(3), 8, line controller의 복구 vx/vy/wz, NONE, NONE
ACTION은 ControlCommandCoordinator가 action_id를 발급한다. ACK 전에는 ACTION을
같은 ID로 반복하고, ACK 뒤에는 command_type=HOLD와 0/0/0을 DONE까지 유지한다.
enabled=false인 기존 호환 호출은 ACTION 대신 controller의 시간 기반 placeholder
속도/정지를 사용한다. 실제 최종 조합은 control_command.cpp에서 결정한다.
*/
