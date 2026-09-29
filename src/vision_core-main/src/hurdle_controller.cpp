// 처리 순서: object_target_extractor의 hurdle 후보를 받아 라인 주행을 덮어쓸 명령을 만든다.
// 실제 잔발/넘기 정책은 외부 모션 패키지의 역할이며, 현재는 상태와 placeholder만 제공한다.

#include "vision_core/hurdle_controller.hpp"

#include "vision_core/config_loader.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
constexpr double kTimeEpsilon = 1e-9;
double SafeDenominator(double value) { return std::max(value, 1e-6); }
} // namespace

HurdleController::HurdleController()
    : HurdleController(LoadDefaultAlgorithmConfig().hurdle) {}

HurdleController::HurdleController(const HurdleConfig &config)
    : config_(config) {}

const char *HurdleController::ModeName(HurdleMode mode) {
  switch (mode) {
  case HurdleMode::kLineFollow: return "LINE_FOLLOW";
  case HurdleMode::kApproach: return "HURDLE_APPROACH";
  case HurdleMode::kTiltCameraDownAndSlow:
    return "HURDLE_CAMERA_TILT_DOWN_AND_SLOW";
  case HurdleMode::kContactWalk: return "HURDLE_CONTACT_WALK";
  case HurdleMode::kCross: return "HURDLE_CROSS";
  case HurdleMode::kReturnCameraToLine:
    return "HURDLE_CAMERA_RETURN_TO_LINE";
  case HurdleMode::kRlStopping: return "HURDLE_RL_STOPPING";
  case HurdleMode::kRecoveryForward: return "HURDLE_RECOV_FORWARD";
  case HurdleMode::kRecoveryDown: return "HURDLE_RECOV_DOWN";
  case HurdleMode::kFailed: return "HURDLE_FAILED";
  }
  return "UNKNOWN";
}

double HurdleController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double HurdleController::LimitRate(double previous, double target,
                                   double delta) {
  return Clamp(target, previous - delta, previous + delta);
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &hurdle_target, int image_width,
    int image_height, double now_sec, const CameraFeedback &camera_feedback) {
  return Compute(hurdle_target, image_width, image_height, now_sec,
                 config_.approach_vx, true, camera_feedback);
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &hurdle_target, int image_width,
    int image_height, double now_sec, double line_vx,
    bool line_reference_valid, const CameraFeedback &camera_feedback) {
  return Compute(hurdle_target, image_width, image_height, now_sec, line_vx,
                 line_reference_valid, camera_feedback,
                 ActionExecutionFeedback{});
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &hurdle_target, int image_width,
    int image_height, double now_sec, double line_vx,
    bool line_reference_valid, const CameraFeedback &camera_feedback,
    const ActionExecutionFeedback &action_feedback) {
  if (line_reference_valid && std::isfinite(line_vx) && line_vx > 0.0) {
    last_tracking_line_vx_ = line_vx;
  }
  if (mode_ == HurdleMode::kLineFollow &&
      now_sec + kTimeEpsilon < ignore_until_sec_) {
    HurdleResult result;
    result.mode = HurdleMode::kLineFollow;
    return result;
  }
  if (mode_ == HurdleMode::kLineFollow && ignore_until_sec_ > 0.0) {
    ignore_until_sec_ = 0.0;
    hit_history_.clear();
    acquire_history_.clear();
    tilt_history_.clear();
    has_smoothed_ = false;
    tracked_ = {};
  }

  const bool camera_observation_valid =
      camera_feedback.actual_mode != CameraMode::kTransition &&
      camera_feedback.settled;
  if (camera_observation_valid) {
    UpdateTracker(hurdle_target, image_width, image_height);
  }
  const int acquire_hits = static_cast<int>(std::count(
      acquire_history_.begin(), acquire_history_.end(), true));
  if (mode_ == HurdleMode::kLineFollow && tracked_.stable && tracked_.visible &&
      acquire_hits >= std::max(1, config_.stable_min_hits)) {
    const double reference_vx =
        last_tracking_line_vx_ > 0.0 ? last_tracking_line_vx_
                                     : config_.approach_vx;
    latched_approach_vx_ =
        reference_vx * Clamp(config_.approach_speed_scale, 0.0, 1.0);
    mode_ = HurdleMode::kApproach;
    state_enter_sec_ = now_sec;
  }
  if (mode_ == HurdleMode::kApproach &&
      lost_count_ >= std::max(1, config_.lost_frames)) {
    mode_ = camera_feedback.actual_mode == CameraMode::kDown
                ? HurdleMode::kRecoveryDown
                : HurdleMode::kRecoveryForward;
    state_enter_sec_ = now_sec;
    recovery_visible_count_ = 0;
  }

  HurdleResult result;
  result.mode = mode_;
  result.tracked = tracked_;
  switch (mode_) {
  case HurdleMode::kLineFollow:
    return result;
  case HurdleMode::kApproach: {
    result.active = true;
    result.command = tracked_.visible ? ComputeApproachCommand()
                                      : MotionCommand{};
    last_command_ = result.command;
    const int hits = static_cast<int>(
        std::count(tilt_history_.begin(), tilt_history_.end(), true));
    if (hits >= std::max(1, config_.tilt_trigger_min_hits)) {
      const double scaled = std::max(0.0, result.command.vx) *
                            Clamp(config_.tilt_walk_speed_scale, 0.0, 1.0);
      latched_tilt_vx_ = Clamp(
          scaled > 0.0 ? scaled : config_.tilt_walk_default_vx,
          0.0, std::max(0.0, config_.tilt_walk_vx_max));
      mode_ = HurdleMode::kTiltCameraDownAndSlow;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kDown;
      result.command = {latched_tilt_vx_, 0.0, 0.0};
      last_command_ = result.command;
    }
    return result;
  }
  case HurdleMode::kRecoveryForward:
  case HurdleMode::kRecoveryDown: {
    const bool recovery_down = mode_ == HurdleMode::kRecoveryDown;
    result.active = true;
    if (recovery_down) result.camera_request = CameraRequest::kDown;
    result.command = ComputeRecoveryCommand();
    last_command_ = result.command;
    if (tracked_.visible) {
      ++recovery_visible_count_;
      if (recovery_visible_count_ >=
          std::max(1, config_.recovery_reacquire_min_hits)) {
        mode_ = recovery_down
                    ? (action_feedback.enabled ? HurdleMode::kRlStopping
                                               : HurdleMode::kContactWalk)
                    : HurdleMode::kApproach;
        state_enter_sec_ = now_sec;
        result.mode = mode_;
        result.camera_request = CameraRequest::kNone;
        result.command = recovery_down ? MotionCommand{}
                                       : ComputeApproachCommand();
        if (recovery_down && !action_feedback.enabled) {
          result.action_request = HurdleActionRequest::kContactWalk;
          result.command = {std::max(0.0, config_.contact_walk_placeholder_vx),
                            0.0, 0.0};
        }
        last_command_ = result.command;
      }
    } else {
      recovery_visible_count_ = 0;
    }
    if (mode_ == HurdleMode::kRecoveryForward ||
        mode_ == HurdleMode::kRecoveryDown) {
      if (now_sec - state_enter_sec_ + kTimeEpsilon >=
          std::max(0.0, config_.recovery_timeout_sec)) {
        mode_ = HurdleMode::kFailed;
        result.mode = mode_;
        result.camera_request = CameraRequest::kNone;
        result.command = {};
        last_command_ = {};
      }
    }
    return result;
  }
  case HurdleMode::kFailed:
    // 실패 상태도 active를 유지해 selector가 Line/Ball 명령으로 넘어가지 않게
    // 한다. 명시적인 Reset() 전까지 정지 상태로 잠긴다.
    result.active = true;
    result.command = {};
    last_command_ = {};
    return result;
  case HurdleMode::kTiltCameraDownAndSlow:
    result.active = true;
    result.camera_request = CameraRequest::kDown;
    result.command = {latched_tilt_vx_, 0.0, 0.0};
    if (camera_feedback.actual_mode == CameraMode::kDown &&
        camera_feedback.settled) {
      mode_ = tracked_.visible
                  ? (action_feedback.enabled ? HurdleMode::kRlStopping
                                             : HurdleMode::kContactWalk)
                  : HurdleMode::kRecoveryDown;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = tracked_.visible ? CameraRequest::kNone
                                               : CameraRequest::kDown;
      if (!tracked_.visible) {
        recovery_visible_count_ = 0;
        result.command = ComputeRecoveryCommand();
      } else if (action_feedback.enabled) {
        result.command = {};
      } else {
        result.action_request = HurdleActionRequest::kContactWalk;
        result.command = {std::max(0.0, config_.contact_walk_placeholder_vx),
                          0.0, 0.0};
      }
    } else if (now_sec - state_enter_sec_ + kTimeEpsilon >=
               std::max(0.0, config_.camera_motion_timeout_sec)) {
      result.command = {};
    }
    return result;
  case HurdleMode::kRlStopping:
    result.active = true;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.rl_stop_duration_sec)) {
      mode_ = HurdleMode::kContactWalk;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = HurdleActionRequest::kContactWalk;
      result.command = {std::max(0.0, config_.contact_walk_placeholder_vx),
                        0.0, 0.0};
    }
    return result;
  case HurdleMode::kContactWalk:
    result.active = true;
    result.action_request = HurdleActionRequest::kContactWalk;
    // 실제 로봇은 STEP_FORWARD_ONE action을 실행한다. 같은 action을
    // velocity-compatible하게 모사하는 MuJoCo에는 이 PRE-P2P 속도를 준다.
    result.command = {std::max(0.0, config_.contact_walk_placeholder_vx),
                      0.0, 0.0};
    if ((action_feedback.enabled && action_feedback.action_done) ||
        (!action_feedback.enabled &&
         now_sec - state_enter_sec_ + kTimeEpsilon >=
             std::max(0.0, config_.contact_walk_placeholder_sec))) {
      mode_ = HurdleMode::kCross;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = HurdleActionRequest::kCross;
      result.command = {};
    }
    return result;
  case HurdleMode::kCross:
    result.active = true;
    result.action_request = HurdleActionRequest::kCross;
    result.command = {};
    if ((action_feedback.enabled && action_feedback.action_done) ||
        (!action_feedback.enabled &&
         now_sec - state_enter_sec_ + kTimeEpsilon >=
             std::max(0.0, config_.cross_placeholder_sec))) {
      mode_ = HurdleMode::kReturnCameraToLine;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = HurdleActionRequest::kNone;
      result.camera_request = CameraRequest::kForward;
    }
    return result;
  case HurdleMode::kReturnCameraToLine:
    result.active = true;
    result.camera_request = CameraRequest::kForward;
    result.command = {};
    if (camera_feedback.actual_mode == CameraMode::kForward &&
        camera_feedback.settled) {
      ignore_until_sec_ =
          now_sec + std::max(0.0, config_.hurdle_ignore_duration_sec);
      ResetToLine(false);
      result = {};
      result.mode = HurdleMode::kLineFollow;
    }
    return result;
  }
  return result;
}

void HurdleController::UpdateTracker(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height) {
  const bool detected = target.has_value() && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    hit_history_.pop_front();
  }

  bool close = false;
  bool acquire = false;
  if (detected) {
    lost_count_ = 0;
    TrackedHurdle observed;
    observed.visible = true;
    const Point2 center = target->center_rectified
                              ? target->rectified_center_px
                              : target->center_px;
    observed.u_norm = Clamp(center.u / SafeDenominator(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(center.v / SafeDenominator(image_height), 0.0, 1.0);
    observed.h_norm = Clamp(
        target->height_px / SafeDenominator(image_height), 0.0, 1.0);
    observed.bottom_norm = Clamp(
        (target->box_px.y + target->box_px.height) /
            SafeDenominator(image_height),
        0.0, 1.0);
    observed.confidence = target->confidence;
    const double raw_v_norm = Clamp(
        target->center_px.v / SafeDenominator(image_height), 0.0, 1.0);
    close = raw_v_norm >= config_.tilt_trigger_v_norm;
    acquire = raw_v_norm >= config_.acquire_min_v_norm;
    last_seen_u_norm_ = observed.u_norm;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      tracked_ = observed;
      has_smoothed_ = true;
    } else {
      tracked_.visible = true;
      tracked_.u_norm = (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm = (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = (1.0 - alpha) * tracked_.h_norm + alpha * observed.h_norm;
      tracked_.bottom_norm =
          (1.0 - alpha) * tracked_.bottom_norm + alpha * observed.bottom_norm;
      tracked_.confidence = observed.confidence;
    }
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }

  tilt_history_.push_back(close);
  while (static_cast<int>(tilt_history_.size()) >
         std::max(1, config_.tilt_trigger_window)) {
    tilt_history_.pop_front();
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
  acquire_history_.push_back(detected && acquire);
  while (static_cast<int>(acquire_history_.size()) >
         std::max(1, config_.stable_window)) {
    acquire_history_.pop_front();
  }
}

MotionCommand HurdleController::ComputeApproachCommand() const {
  const double u_error =
      (tracked_.u_norm - config_.target_u_norm) / 0.5;
  const double wz_raw =
      -std::abs(config_.approach_wz_max) *
      std::tanh(config_.approach_wz_gain * u_error);
  MotionCommand command;
  command.vx = std::max(0.0, latched_approach_vx_);
  command.wz = LimitRate(last_command_.wz, wz_raw, config_.approach_dw_max);
  return command;
}

MotionCommand HurdleController::ComputeRecoveryCommand() const {
  MotionCommand command;
  command.vx = std::max(0.0, config_.recovery_forward_vx);
  const double horizontal_error = last_seen_u_norm_ - 0.50;
  if (std::abs(horizontal_error) >
      std::max(0.0, config_.recovery_center_tolerance_norm)) {
    command.wz = horizontal_error > 0.0
                     ? -std::abs(config_.recovery_turn_wz)
                     : std::abs(config_.recovery_turn_wz);
  }
  return command;
}

void HurdleController::ResetToLine(bool clear_ignore) {
  mode_ = HurdleMode::kLineFollow;
  state_enter_sec_ = 0.0;
  hit_history_.clear();
  acquire_history_.clear();
  tilt_history_.clear();
  lost_count_ = 0;
  recovery_visible_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
  last_command_ = {};
  latched_approach_vx_ = 0.0;
  latched_tilt_vx_ = 0.0;
  last_seen_u_norm_ = 0.50;
  if (clear_ignore) last_tracking_line_vx_ = 0.0;
  if (clear_ignore) ignore_until_sec_ = 0.0;
}

void HurdleController::Reset() { ResetToLine(true); }

} // namespace vision_core

/*
ROS 통합 명령표 (ActionExecutionFeedback.enabled=true)
형식: MODE -> command_type, mission, mission_phase, velocity, action, camera

LINE_FOLLOW(0)
  -> VELOCITY, LINE(1), 0, line controller의 vx/vy/wz, NONE, NONE
HURDLE_APPROACH(1)
  -> VELOCITY, HURDLE(4), 1, ComputeApproachCommand(), NONE, NONE
HURDLE_RECOV_FORWARD(7)
  -> VELOCITY, HURDLE(4), 7, ComputeRecoveryCommand(), NONE, NONE
HURDLE_RECOV_DOWN(8)
  -> VELOCITY, HURDLE(4), 8, ComputeRecoveryCommand(), NONE, DOWN(1)
HURDLE_FAILED(9)
  -> VELOCITY, HURDLE(4), 9, 0/0/0, NONE, NONE
HURDLE_CAMERA_TILT_DOWN_AND_SLOW(2)
  -> VELOCITY, HURDLE(4), 2, latched_tilt_vx/0/0, NONE, DOWN(1)
HURDLE_RL_STOPPING(6)
  -> VELOCITY, HURDLE(4), 6, 0/0/0, NONE, NONE
HURDLE_CONTACT_WALK(3)
  -> ACTION, HURDLE(4), 3, 0/0/0, STEP_FORWARD_ONE(10), NONE
HURDLE_CROSS(4)
  -> ACTION, HURDLE(4), 4, 0/0/0, HUDDLE(16), NONE
HURDLE_CAMERA_RETURN_TO_LINE(5)
  -> VELOCITY, HURDLE(4), 5, 0/0/0, NONE, FORWARD(2)
ACTION은 ControlCommandCoordinator가 action_id를 발급한다. ACK 전에는 ACTION을
같은 ID로 반복하고, ACK 뒤에는 command_type=HOLD와 0/0/0을 DONE까지 유지한다.
enabled=false인 기존 호환 호출은 ACTION 대신 controller의 시간 기반 placeholder
속도/정지를 사용한다. 실제 최종 조합은 control_command.cpp에서 결정한다.
*/
