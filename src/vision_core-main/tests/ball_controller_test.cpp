#include "vision_core/ball_controller.hpp"
#include "vision_core/c_api.h"
#include "vision_core/config_loader.hpp"
#include "vision_core/motion_command_selector.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <optional>

namespace {

using vision_core::BallConfig;
using vision_core::BallController;
using vision_core::BallActionRequest;
using vision_core::BallMode;
using vision_core::BallResult;
using vision_core::ActionExecutionFeedback;
using vision_core::CameraFeedback;
using vision_core::CameraMode;
using vision_core::CameraRequest;
using vision_core::Features;
using vision_core::MotionCommand;
using vision_core::ObjectTarget;

constexpr double kEps = 1e-9;

ObjectTarget Target(double u_norm, double v_norm, double h_norm = 0.10,
                    int width = 100, int height = 100) {
  ObjectTarget target;
  target.class_id = 1;
  target.confidence = 1.0;
  target.center_px = {u_norm * width, v_norm * height};
  target.rectified_center_px = target.center_px;
  target.width_px = h_norm * height;
  target.height_px = h_norm * height;
  target.area_px = target.width_px * target.height_px;
  return target;
}

CameraFeedback Forward() { return {CameraMode::kForward, true}; }
CameraFeedback Down() { return {CameraMode::kDown, true}; }
CameraFeedback Moving() { return {CameraMode::kTransition, false}; }

void ExpectNear(double actual, double expected) {
  if (std::abs(actual - expected) > kEps) {
    std::cerr << "expected " << expected << ", got " << actual << '\n';
  }
  assert(std::abs(actual - expected) <= kEps);
}

void TestFarSpeedAndRollingWindowTrigger() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.smooth_alpha = 1.0;
  cfg.tilt_down_window = 5;
  cfg.tilt_down_min_hits = 3;
  cfg.far_speed_scale = 0.9;
  cfg.tilt_walk_speed_scale = 0.5;
  cfg.tilt_walk_vx_max = 1.0;
  BallController controller(cfg);

  auto high = Target(0.90, 0.80);
  auto low = Target(0.50, 0.60, 0.95);  // huge bbox must not trigger tilt

  auto result = controller.Compute(
      high, 100, 100, 0.00, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  ExpectNear(result.command.vx, 0.8 * cfg.far_speed_scale);

  result = controller.Compute(
      low, 100, 100, 0.02, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  ExpectNear(result.command.vx, 0.8 * cfg.far_speed_scale);

  result = controller.Compute(
      high, 100, 100, 0.04, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  result = controller.Compute(
      std::nullopt, 100, 100, 0.06, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  // Background line control has now entered recovery and reports a positive
  // coast speed. FAR must keep the normal TRACK speed captured on entry.
  result = controller.Compute(
      high, 100, 100, 0.08, 0.12, false, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(result.camera_request == CameraRequest::kDown);
  ExpectNear(result.command.vx,
             std::min(cfg.tilt_walk_vx_max,
                      0.8 * cfg.far_speed_scale * cfg.tilt_walk_speed_scale));
  ExpectNear(result.command.wz, 0.0);
}

void TestPositiveRecoveryCannotOverwriteTrackingReference() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 3;
  cfg.stable_min_hits = 3;
  cfg.smooth_alpha = 1.0;
  cfg.tilt_down_min_hits = 99;
  cfg.far_speed_scale = 0.9;
  BallController controller(cfg);
  auto target = Target(0.50, 0.50);

  auto result = controller.Compute(
      target, 100, 100, 0.00, 0.8, true, Forward());
  assert(result.mode == BallMode::kLineFollow);
  result = controller.Compute(
      target, 100, 100, 0.02, 0.12, false, Forward());
  assert(result.mode == BallMode::kLineFollow);
  result = controller.Compute(
      target, 100, 100, 0.04, 0.12, false, Forward());
  assert(result.mode == BallMode::kApproachBall);
  ExpectNear(result.command.vx, 0.8 * cfg.far_speed_scale);
}

void TestRecoveryAloneIsNotAForwardReference() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_min_hits = 99;
  BallController controller(cfg);

  const auto result = controller.Compute(
      Target(0.50, 0.50), 100, 100, 0.0, 0.12, false, Forward());
  assert(result.mode == BallMode::kApproachBall);
  // With no normal TRACK sample, fail safe instead of treating RECOV as the
  // requested baseline walking speed.
  ExpectNear(result.command.vx, 0.0);
}

void TestDefaultStableFramesCountTowardTiltAndRawScreenWins() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.smooth_alpha = 1.0;
  BallController controller(cfg);
  const double upper_v_norm =
      std::max(0.01, std::min(cfg.upper_acquire_v_norm,
                              cfg.tilt_down_v_norm) - 0.10);
  const double lower_v_norm =
      std::min(0.99, std::max(cfg.upper_acquire_v_norm,
                              cfg.tilt_down_v_norm) + 0.05);
  auto upper = Target(0.50, upper_v_norm);
  auto lower = Target(0.50, lower_v_norm);

  // 화면 아래쪽에서만 생긴 검출은 YAML의 안정화 조건을 만족해도
  // 공 미션을 시작하지 않는다.
  for (int frame = 0; frame < cfg.stable_window; ++frame) {
    const auto result = controller.Compute(
        lower, 100, 100, 0.02 * frame, 0.8, Forward());
    assert(result.mode == BallMode::kLineFollow);
  }

  // 먼저 화면 위쪽에서 YAML의 안정 획득 조건을 만족한 뒤,
  // tilt_down_v_norm 아래 영역에서 tilt_down_min_hits를 만족해야
  // 카메라를 내린다.
  BallController armed_controller(cfg);
  BallResult result;
  for (int frame = 0; frame < cfg.stable_min_hits; ++frame) {
    result = armed_controller.Compute(
        upper, 100, 100, 0.20 + 0.02 * frame, 0.8, Forward());
  }
  assert(result.mode == BallMode::kApproachBall);
  for (int frame = 0; frame < cfg.tilt_down_min_hits - 1; ++frame) {
    result = armed_controller.Compute(
        lower, 100, 100, 0.40 + 0.02 * frame, 0.8, Forward());
    assert(result.mode == BallMode::kApproachBall);
  }
  result = armed_controller.Compute(
      lower, 100, 100,
      0.40 + 0.02 * (cfg.tilt_down_min_hits - 1), 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);

  BallConfig raw_cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  raw_cfg.upper_acquire_v_norm = 1.01;
  raw_cfg.stable_window = 1;
  raw_cfg.stable_min_hits = 1;
  raw_cfg.tilt_down_window = 1;
  raw_cfg.tilt_down_min_hits = 1;
  raw_cfg.smooth_alpha = 1.0;
  BallController raw_controller(raw_cfg);
  const double raw_upper_v_norm =
      std::max(0.01, raw_cfg.tilt_down_v_norm - 0.10);
  const double raw_lower_v_norm =
      std::min(0.99, raw_cfg.tilt_down_v_norm + 0.05);
  auto rectified_only = Target(0.50, raw_upper_v_norm);
  rectified_only.center_rectified = true;
  rectified_only.rectified_center_px.v = raw_lower_v_norm * 100.0;
  result = raw_controller.Compute(
      rectified_only, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kApproachBall);

  auto raw_below = Target(0.50, raw_lower_v_norm);
  raw_below.center_rectified = true;
  raw_below.rectified_center_px.v = raw_upper_v_norm * 100.0;
  result = raw_controller.Compute(
      raw_below, 100, 100, 0.02, 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
}

void TestBallPickupPlaceholderSequenceAndCooldown() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.far_speed_scale = 1.0;
  cfg.tilt_walk_speed_scale = 0.5;
  cfg.tilt_walk_vx_max = 1.0;
  cfg.fine_adjust_placeholder_vx = 0.15;
  cfg.fine_adjust_placeholder_duration_sec = 1.0;
  cfg.pickup_placeholder_duration_sec = 3.0;
  cfg.pickup_verification_placeholder_sec = 0.0;
  cfg.stand_up_placeholder_sec = 0.0;
  cfg.pickup_success_missing_frames = 1;
  cfg.post_pickup_back_away_sec = 0.0;
  cfg.ball_ignore_duration_sec = 30.0;
  BallController controller(cfg);
  auto target = Target(0.50, 0.80);

  auto result = controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  ExpectNear(result.command.vx,
             std::min(cfg.tilt_walk_vx_max,
                      0.8 * cfg.far_speed_scale * cfg.tilt_walk_speed_scale));

  result = controller.Compute(target, 100, 100, 0.1, 0.8, Moving());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(result.camera_request == CameraRequest::kDown);

  result = controller.Compute(
      target, 100, 100, 0.2, 0.8, {CameraMode::kDown, false});
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = controller.Compute(target, 100, 100, 0.3, 0.8, Down());
  assert(result.mode == BallMode::kFineAdjustForPickup);
  assert(result.camera_request == CameraRequest::kNone);
  ExpectNear(result.command.vx, cfg.fine_adjust_placeholder_vx);

  result = controller.Compute(target, 100, 100, 1.29, 0.8, Down());
  assert(result.mode == BallMode::kFineAdjustForPickup);
  result = controller.Compute(target, 100, 100, 1.30, 0.8, Down());
  assert(result.mode == BallMode::kPickupBall);
  assert(result.reached_pickup_pose);
  assert(result.camera_request == CameraRequest::kNone);
  ExpectNear(result.command.vx, 0.0);

  result = controller.Compute(target, 100, 100, 4.29, 0.8, Down());
  assert(result.mode == BallMode::kPickupBall);
  result = controller.Compute(target, 100, 100, 4.30, 0.8, Down());
  assert(result.mode == BallMode::kVerifyPickup);
  assert(result.camera_request == CameraRequest::kNone);
  assert(result.active);

  result = controller.Compute(std::nullopt, 100, 100, 4.32, 0.8, Down());
  assert(result.mode == BallMode::kVerifyPickupObservation);
  result = controller.Compute(std::nullopt, 100, 100, 4.34, 0.8, Down());
  assert(result.mode == BallMode::kStandUpAfterPickup);
  assert(result.has_ball);
  assert(controller.HasBall());
  result = controller.Compute(std::nullopt, 100, 100, 4.36, 0.8, Down());
  assert(result.mode == BallMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);
  result = controller.Compute(std::nullopt, 100, 100, 4.40, 0.8, Moving());
  assert(result.mode == BallMode::kReturnCameraToLine);
  result = controller.Compute(std::nullopt, 100, 100, 4.50, 0.8, false,
                              Forward());
  assert(result.mode == BallMode::kPostPickupBackAway);
  assert(result.command.vx < 0.0);
  result = controller.Compute(std::nullopt, 100, 100, 4.52, 0.8, false,
                              Forward());
  assert(result.mode == BallMode::kPostPickupLineRecovery);
  assert(!result.active);
  result = controller.Compute(std::nullopt, 100, 100, 4.54, 0.8, true,
                              Forward());
  assert(result.mode == BallMode::kLineFollow);
  assert(result.camera_request == CameraRequest::kNone);
  assert(!result.active);
  assert(result.has_ball);

  // 슛 완료에 해당하는 외부 상태 갱신 뒤에도 cooldown 동안 같은 공은 무시한다.
  controller.SetHasBall(false);
  result = controller.Compute(
      target, 100, 100, 34.53, 0.6, true, Forward());
  assert(result.mode == BallMode::kLineFollow);
  assert(!result.active);
  // At expiry, tracking starts fresh and can enter again with a 1-frame test
  // stability window.
  result = controller.Compute(
      target, 100, 100, 34.54, 0.12, false, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  ExpectNear(result.command.vx,
             std::min(cfg.tilt_walk_vx_max,
                      0.6 * cfg.far_speed_scale * cfg.tilt_walk_speed_scale));
}

void TestFeedbackSequenceKeepsVerifyPickupInActionMode() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.rl_stop_duration_sec = 0.5;
  cfg.pickup_max_attempts = 3;
  cfg.pickup_success_missing_frames = 1;
  cfg.post_pickup_back_away_sec = 0.0;
  cfg.ball_ignore_duration_sec = 0.0;
  BallController controller(cfg);
  const auto target = Target(0.50, 0.80);
  const ActionExecutionFeedback waiting{true, false, false};
  const ActionExecutionFeedback done{true, true, false};

  auto result = controller.Compute(
      target, 100, 100, 0.0, 0.8, true, Forward(), waiting);
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(result.command.vx > 0.0);

  result = controller.Compute(
      target, 100, 100, 0.1, 0.8, true, Down(), waiting);
  assert(result.mode == BallMode::kRlStoppingForPickup);
  ExpectNear(result.command.vx, 0.0);

  result = controller.Compute(
      target, 100, 100, 0.6, 0.8, true, Down(), waiting);
  assert(result.mode == BallMode::kFineAdjustForPickup);
  assert(result.action_request == BallActionRequest::kFineAdjustForward);
  ExpectNear(result.command.vx, cfg.fine_adjust_placeholder_vx);

  result = controller.Compute(
      target, 100, 100, 0.7, 0.8, true, Down(), done);
  assert(result.mode == BallMode::kPickupBall);
  assert(result.action_request == BallActionRequest::kPickup);

  result = controller.Compute(
      target, 100, 100, 0.8, 0.8, true, Down(), done);
  assert(result.mode == BallMode::kVerifyPickup);
  assert(result.action_request == BallActionRequest::kVerifyPickup);

  result = controller.Compute(
      target, 100, 100, 0.9, 0.8, true, Down(), waiting);
  assert(result.mode == BallMode::kVerifyPickup);
  assert(result.action_request == BallActionRequest::kVerifyPickup);

  result = controller.Compute(
      target, 100, 100, 1.0, 0.8, true, Down(), done);
  assert(result.mode == BallMode::kVerifyPickupObservation);
  assert(result.action_request == BallActionRequest::kNone);

  // 공이 계속 보이면 최대 3회까지 PICKUP을 다시 요청한다.
  result = controller.Compute(
      target, 100, 100, 1.1, 0.8, true, Down(), waiting);
  assert(result.mode == BallMode::kPickupBall);
  assert(result.pickup_attempt_count == 2);
  result = controller.Compute(
      target, 100, 100, 1.2, 0.8, true, Down(), done);
  assert(result.mode == BallMode::kVerifyPickup);
  result = controller.Compute(
      target, 100, 100, 1.3, 0.8, true, Down(), done);
  assert(result.mode == BallMode::kVerifyPickupObservation);
  result = controller.Compute(
      target, 100, 100, 1.4, 0.8, true, Down(), waiting);
  assert(result.mode == BallMode::kPickupBall);
  assert(result.pickup_attempt_count == 3);
  result = controller.Compute(
      target, 100, 100, 1.5, 0.8, true, Down(), done);
  assert(result.mode == BallMode::kVerifyPickup);
  result = controller.Compute(
      target, 100, 100, 1.6, 0.8, true, Down(), done);
  assert(result.mode == BallMode::kVerifyPickupObservation);
  result = controller.Compute(
      target, 100, 100, 1.7, 0.8, true, Down(), waiting);
  assert(result.mode == BallMode::kStandUpAfterPickup);
  assert(result.action_request == BallActionRequest::kStandUp);
  assert(!result.has_ball);
  assert(result.pickup_failed);
  assert(controller.PickupFailed());
  assert(controller.PickupAttemptCount() == 3);

  result = controller.Compute(
      std::nullopt, 100, 100, 1.8, 0.8, false, Down(), done);
  assert(result.mode == BallMode::kReturnCameraToLine);
  result = controller.Compute(
      std::nullopt, 100, 100, 1.9, 0.8, false, Forward(), waiting);
  assert(result.mode == BallMode::kPostPickupBackAway);
  result = controller.Compute(
      std::nullopt, 100, 100, 2.0, 0.8, false, Forward(), waiting);
  assert(result.mode == BallMode::kPostPickupLineRecovery);
  result = controller.Compute(
      std::nullopt, 100, 100, 2.1, 0.8, true, Forward(), waiting);
  assert(result.mode == BallMode::kLineFollow);
  assert(result.pickup_failed);

  // 다음 공이 안정 검출되어 새 Ball 미션이 시작되면 이전 실패 플래그를 지운다.
  result = controller.Compute(
      target, 100, 100, 2.2, 0.8, true, Forward(), waiting);
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(!result.pickup_failed);
  assert(!controller.PickupFailed());
}

void TestCameraTimeoutAndLegacyTimedFeedback() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.camera_motion_timeout_sec = 0.5;
  BallController controller(cfg);
  auto target = Target(0.50, 0.80);

  auto result = controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = controller.Compute(target, 100, 100, 0.49, 0.8, Moving());
  assert(result.command.vx > 0.0);
  result = controller.Compute(target, 100, 100, 0.50, 0.8, Moving());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(result.camera_request == CameraRequest::kDown);
  ExpectNear(result.command.vx, 0.0);
  result = controller.Compute(target, 100, 100, 0.60, 0.8, Down());
  assert(result.mode == BallMode::kFineAdjustForPickup);

  BallConfig legacy_cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  legacy_cfg.upper_acquire_v_norm = 1.01;
  legacy_cfg.stable_window = 1;
  legacy_cfg.stable_min_hits = 1;
  legacy_cfg.tilt_down_window = 1;
  legacy_cfg.tilt_down_min_hits = 1;
  legacy_cfg.camera_tilt_duration_sec = 0.50;
  legacy_cfg.camera_settle_sec = 0.25;
  BallController legacy(legacy_cfg);
  result = legacy.Compute(target, 100, 100, 0.0, 0.8);
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = legacy.Compute(target, 100, 100, 0.74, 0.8);
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = legacy.Compute(target, 100, 100, 0.75, 0.8);
  assert(result.mode == BallMode::kFineAdjustForPickup);
}

void TestLostFrameLimitIsExact() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_min_hits = 99;
  cfg.lost_frames = 2;
  BallController controller(cfg);
  auto target = Target(0.50, 0.50);

  auto result = controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kApproachBall);
  result = controller.Compute(std::nullopt, 100, 100, 0.1, 0.8, Forward());
  assert(result.mode == BallMode::kApproachBall);
  result = controller.Compute(
      std::nullopt, 100, 100, 0.2, 0.6, true, Forward());
  assert(result.mode == BallMode::kBallRecoveryForward);
  assert(result.active);
  ExpectNear(result.command.vx, cfg.recovery_forward_vx);
  ExpectNear(result.command.wz, 0.0);
  result = controller.Compute(
      target, 100, 100, 0.3, 0.12, false, Forward());
  assert(result.mode == BallMode::kBallRecoveryForward);
  result = controller.Compute(
      target, 100, 100, 0.4, 0.12, false, Forward());
  assert(result.mode == BallMode::kBallRecoveryForward);
  result = controller.Compute(
      target, 100, 100, 0.5, 0.12, false, Forward());
  assert(result.mode == BallMode::kApproachBall);
  // 최초 정상 line vx에 YAML의 원거리 접근 비율을 적용한다.
  ExpectNear(result.command.vx, 0.80 * cfg.far_speed_scale);

  // 우측에서 놓친 공은 BALL_RECOV 동안 우측으로 제자리 회전한다.
  BallController turn_controller(cfg);
  result = turn_controller.Compute(
      Target(0.80, 0.50), 100, 100, 0.0, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  turn_controller.Compute(
      std::nullopt, 100, 100, 0.1, 0.8, true, Forward());
  result = turn_controller.Compute(
      std::nullopt, 100, 100, 0.2, 0.8, true, Forward());
  assert(result.mode == BallMode::kBallRecoveryForward);
  ExpectNear(result.command.vx, 0.0);
  ExpectNear(result.command.wz, -cfg.recovery_turn_wz);

  result = turn_controller.Compute(
      std::nullopt, 100, 100,
      0.2 + cfg.recovery_timeout_sec, 0.8, true, Forward());
  assert(result.mode == BallMode::kLineFollow);
  assert(!result.active);
}

void TestMovingCameraDropsDetectionsAndDownRecoveryStaysDown() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.lost_frames = 2;
  cfg.recovery_reacquire_min_hits = 2;
  BallController controller(cfg);
  auto target = Target(0.50, 0.80);

  // 이동 중 처음 나타난 검출은 공 획득 이력에 들어가지 않는다.
  auto result = controller.Compute(
      target, 100, 100, 0.0, 0.8, true, Moving());
  assert(result.mode == BallMode::kLineFollow);
  result = controller.Compute(
      std::nullopt, 100, 100, 0.1, 0.8, true, Forward());
  assert(result.mode == BallMode::kLineFollow);

  // 정면에서 정상 획득해 카메라를 내리는 동안의 검출/소실도 버린다.
  result = controller.Compute(
      target, 100, 100, 0.2, 0.8, true, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = controller.Compute(
      std::nullopt, 100, 100, 0.3, 0.8, true, Moving());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);

  // 아래에 도착했는데 공이 없으면 카메라를 유지한 BALL_RECOV_DOWN.
  result = controller.Compute(
      std::nullopt, 100, 100, 0.4, 0.8, true, Down());
  assert(result.mode == BallMode::kBallRecoveryDown);
  assert(result.camera_request == CameraRequest::kDown);
  result = controller.Compute(
      target, 100, 100, 0.5, 0.8, true, Down());
  assert(result.mode == BallMode::kBallRecoveryDown);
  result = controller.Compute(
      target, 100, 100, 0.6, 0.8, true, Down());
  assert(result.mode == BallMode::kFineAdjustForPickup);

  // 하향 복구가 실패하면 라인 제어로 바로 넘기지 않고 카메라부터 올린다.
  BallController timeout_controller(cfg);
  timeout_controller.Compute(
      target, 100, 100, 0.0, 0.8, true, Forward());
  result = timeout_controller.Compute(
      std::nullopt, 100, 100, 0.1, 0.8, true, Down());
  assert(result.mode == BallMode::kBallRecoveryDown);
  result = timeout_controller.Compute(
      std::nullopt, 100, 100,
      0.1 + cfg.recovery_timeout_sec, 0.8, true, Down());
  assert(result.mode == BallMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);
}

void TestResetClearsCooldownAndState() {
  BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  cfg.upper_acquire_v_norm = 1.01;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.fine_adjust_placeholder_duration_sec = 0.0;
  cfg.pickup_placeholder_duration_sec = 0.0;
  cfg.pickup_verification_placeholder_sec = 0.0;
  cfg.stand_up_placeholder_sec = 0.0;
  cfg.ball_ignore_duration_sec = 30.0;
  BallController controller(cfg);
  auto target = Target(0.5, 0.8);
  controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  controller.Compute(target, 100, 100, 0.1, 0.8, Down());
  controller.Compute(target, 100, 100, 0.2, 0.8, Down());
  controller.Compute(target, 100, 100, 0.3, 0.8, Down());
  controller.Reset();
  const auto result = controller.Compute(
      target, 100, 100, 0.4, 0.4, true, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  const double expected_tilt_vx = std::min(
      cfg.tilt_walk_vx_max,
      0.4 * cfg.far_speed_scale * cfg.tilt_walk_speed_scale);
  ExpectNear(result.command.vx, expected_tilt_vx);
}

void TestSelectorKeepsLineCandidateWhileBallActive() {
  vision_core::LineVelocityController line_controller;
  Features features;
  features.n_visible = 5.0;
  vision_core::BallResult ball;
  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.2, 0.0, 0.1};
  const auto selected =
      vision_core::SelectMotionCommand(ball, features, line_controller);
  assert(selected.source == vision_core::CommandSource::kBall);
  ExpectNear(selected.command.vx, 0.2);

  const MotionCommand explicit_line{0.7, 0.0, -0.2};
  const auto pure = vision_core::SelectMotionCommand(ball, explicit_line);
  ExpectNear(pure.command.vx, 0.2);

  ball.active = false;
  const auto line_selected = vision_core::SelectMotionCommand(ball, explicit_line);
  ExpectNear(line_selected.command.vx, 0.7);
}

void TestMissionSelectorPriority() {
  vision_core::BallResult ball;
  vision_core::HurdleResult hurdle;
  vision_core::GoalResult goal;
  const MotionCommand line{0.7, 0.0, -0.2};

  hurdle.active = true;
  hurdle.mode = vision_core::HurdleMode::kApproach;
  hurdle.command = {0.3, 0.0, 0.1};
  auto selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kHurdle);
  ExpectNear(selected.command.vx, 0.3);

  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.2, 0.0, -0.1};
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kHurdle);
  ExpectNear(selected.command.vx, 0.3);

  ball.has_ball = true;
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kBall);
  ExpectNear(selected.command.vx, 0.2);

  goal.active = true;
  goal.mode = vision_core::GoalMode::kApproach;
  goal.command = {0.1, 0.0, 0.0};
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kGoal);
  ExpectNear(selected.command.vx, 0.1);

  goal.active = false;
  goal.mode = vision_core::GoalMode::kHeadingRecovery;
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kLine);
  ExpectNear(selected.command.vx, 0.7);

  // 공 집기 후 라인 재획득 중에는 hurdle보다 line 복구 명령을 우선한다.
  goal = {};
  ball.active = false;
  ball.has_ball = true;
  ball.mode = BallMode::kPostPickupLineRecovery;
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kLine);
  ExpectNear(selected.command.vx, 0.0);
  ExpectNear(selected.command.wz, -0.2);
}

void TestCApiV2V3LayoutAndPrecomputedSelector() {
  const BallConfig cfg = vision_core::LoadDefaultAlgorithmConfig().ball;
  VisionBallControllerHandle handle = vision_ball_controller_create();
  assert(handle != nullptr);
  VisionObjectTarget target{};
  target.valid = 1;
  target.class_id = 1;
  target.confidence = 1.0;
  target.x = 45.0;
  const double lower_v_norm =
      std::min(0.99, std::max(cfg.upper_acquire_v_norm,
                              cfg.tilt_down_v_norm) + 0.05);
  target.y = lower_v_norm * 100.0 - 5.0;
  target.width = 10.0;
  target.height = 10.0;
  target.center_u = 50.0;
  target.center_v = lower_v_norm * 100.0;
  VisionObjectTarget upper_target = target;
  const double upper_v_norm =
      std::max(0.01, std::min(cfg.upper_acquire_v_norm,
                              cfg.tilt_down_v_norm) - 0.10);
  upper_target.y = upper_v_norm * 100.0 - 5.0;
  upper_target.center_v = upper_v_norm * 100.0;

  VisionBallResult result{};
  for (int frame = 0; frame < cfg.stable_min_hits; ++frame) {
    result = vision_ball_controller_compute_v2(
        handle, upper_target, 100, 100, 0.02 * frame,
        frame == 0 ? 0.8 : 0.0,
        static_cast<int>(CameraMode::kForward), 1);
  }
  for (int frame = 0; frame < cfg.tilt_down_min_hits; ++frame) {
    result = vision_ball_controller_compute_v2(
        handle, target, 100, 100, 0.20 + 0.02 * frame, 0.0,
        static_cast<int>(CameraMode::kForward), 1);
  }
  assert(result.active == 1);
  assert(result.mode ==
         static_cast<int>(BallMode::kTiltCameraDownAndApproach));
  assert(result.request_camera_down == 1);
  const double expected_tilt_vx = std::min(
      cfg.tilt_walk_vx_max,
      0.8 * cfg.far_speed_scale * cfg.tilt_walk_speed_scale);
  ExpectNear(result.vx, expected_tilt_vx);

  vision_ball_controller_reset(handle);
  for (int frame = 0; frame < cfg.stable_min_hits; ++frame) {
    result = vision_ball_controller_compute_v3(
        handle, upper_target, 100, 100, 0.02 * frame,
        frame == 0 ? 0.8 : 0.12, frame == 0 ? 1 : 0,
        static_cast<int>(CameraMode::kForward), 1);
  }
  for (int frame = 0; frame < cfg.tilt_down_min_hits; ++frame) {
    result = vision_ball_controller_compute_v3(
        handle, target, 100, 100, 0.20 + 0.02 * frame,
        0.12, 0, static_cast<int>(CameraMode::kForward), 1);
  }
  assert(result.active == 1);
  assert(result.mode ==
         static_cast<int>(BallMode::kTiltCameraDownAndApproach));
  ExpectNear(result.vx, expected_tilt_vx);

  VisionBallResult active_ball{};
  active_ball.active = 1;
  active_ball.mode = static_cast<int>(BallMode::kApproachBall);
  active_ball.vx = 0.2;
  active_ball.wz = 0.1;
  auto selected = vision_select_motion_command_v2(
      active_ball, 0.7, 0.0, -0.2);
  ExpectNear(selected.vx, 0.2);
  assert(selected.source == 1);

  active_ball.active = 0;
  selected = vision_select_motion_command_v2(
      active_ball, 0.7, 0.0, -0.2);
  ExpectNear(selected.vx, 0.7);
  ExpectNear(selected.wz, -0.2);
  assert(selected.source == 0);
  vision_ball_controller_destroy(handle);
}

}  // namespace

int main() {
  TestFarSpeedAndRollingWindowTrigger();
  TestPositiveRecoveryCannotOverwriteTrackingReference();
  TestRecoveryAloneIsNotAForwardReference();
  TestDefaultStableFramesCountTowardTiltAndRawScreenWins();
  TestBallPickupPlaceholderSequenceAndCooldown();
  TestFeedbackSequenceKeepsVerifyPickupInActionMode();
  TestCameraTimeoutAndLegacyTimedFeedback();
  TestLostFrameLimitIsExact();
  TestMovingCameraDropsDetectionsAndDownRecoveryStaysDown();
  TestResetClearsCooldownAndState();
  TestSelectorKeepsLineCandidateWhileBallActive();
  TestMissionSelectorPriority();
  TestCApiV2V3LayoutAndPrecomputedSelector();
  std::cout << "ball controller tests passed\n";
  return 0;
}
