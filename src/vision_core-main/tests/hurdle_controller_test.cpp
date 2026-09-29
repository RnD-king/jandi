#include "vision_core/hurdle_controller.hpp"
#include "vision_core/config_loader.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>

namespace {

using vision_core::CameraFeedback;
using vision_core::CameraMode;
using vision_core::CameraRequest;
using vision_core::ActionExecutionFeedback;
using vision_core::HurdleActionRequest;
using vision_core::HurdleConfig;
using vision_core::HurdleController;
using vision_core::HurdleMode;
using vision_core::HurdleResult;
using vision_core::ObjectTarget;

ObjectTarget Target(double u_norm, double bottom_norm, double h_norm = 0.20) {
  ObjectTarget target;
  target.class_id = 4;
  target.confidence = 1.0;
  target.width_px = 20.0;
  target.height_px = h_norm * 100.0;
  target.box_px = {u_norm * 100.0 - 10.0,
                   bottom_norm * 100.0 - target.height_px,
                   target.width_px, target.height_px};
  target.center_px = {u_norm * 100.0,
                      target.box_px.y + 0.5 * target.height_px};
  target.rectified_center_px = target.center_px;
  return target;
}

CameraFeedback Forward() { return {CameraMode::kForward, true}; }
CameraFeedback Down() { return {CameraMode::kDown, true}; }
CameraFeedback Moving() { return {CameraMode::kTransition, false}; }

void TestHurdlePlaceholderSequence() {
  HurdleConfig cfg = vision_core::LoadDefaultAlgorithmConfig().hurdle;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_trigger_window = 1;
  cfg.tilt_trigger_min_hits = 1;
  cfg.contact_walk_placeholder_sec = 2.0;
  cfg.cross_placeholder_sec = 3.0;
  HurdleController controller(cfg);
  const auto acquire_hurdle = Target(0.60, 0.70, 0.20);
  const auto hurdle = Target(0.60, 0.90, 0.20);

  auto result = controller.Compute(acquire_hurdle, 100, 100, 0.0, Forward());
  assert(result.mode == HurdleMode::kApproach);
  result = controller.Compute(hurdle, 100, 100, 0.02, Forward());
  assert(result.mode == HurdleMode::kTiltCameraDownAndSlow);
  assert(result.camera_request == CameraRequest::kDown);
  assert(result.command.vx > 0.0);

  result = controller.Compute(hurdle, 100, 100, 0.1, Moving());
  assert(result.mode == HurdleMode::kTiltCameraDownAndSlow);
  result = controller.Compute(hurdle, 100, 100, 0.2, Down());
  assert(result.mode == HurdleMode::kContactWalk);
  assert(result.action_request == HurdleActionRequest::kContactWalk);

  result = controller.Compute(hurdle, 100, 100, 2.19, Down());
  assert(result.mode == HurdleMode::kContactWalk);
  result = controller.Compute(hurdle, 100, 100, 2.20, Down());
  assert(result.mode == HurdleMode::kCross);
  assert(result.action_request == HurdleActionRequest::kCross);
  assert(std::abs(result.command.vx) < 1e-9);

  result = controller.Compute(hurdle, 100, 100, 5.20, Down());
  assert(result.mode == HurdleMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);
  result = controller.Compute(hurdle, 100, 100, 5.30, Forward());
  assert(result.mode == HurdleMode::kLineFollow);
  assert(!result.active);
}

void TestHurdleUsesBallApproachNumbers() {
  HurdleConfig cfg = vision_core::LoadDefaultAlgorithmConfig().hurdle;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_trigger_window = 1;
  cfg.tilt_trigger_min_hits = 1;
  HurdleController controller(cfg);

  const auto centered = Target(0.50, 0.70, 0.20);
  auto result =
      controller.Compute(centered, 100, 100, 0.0, 0.80, true, Forward());
  assert(result.mode == HurdleMode::kApproach);
  assert(std::abs(result.command.vx - 0.60) < 1e-9);
  assert(std::abs(result.command.wz) < 1e-9);

  controller.Reset();
  controller.Compute(centered, 100, 100, 0.1, 0.80, true, Forward());
  const auto down_trigger = Target(0.50, 0.85, 0.20);
  result = controller.Compute(
      down_trigger, 100, 100, 0.2, 0.80, true, Forward());
  assert(result.mode == HurdleMode::kTiltCameraDownAndSlow);
  assert(std::abs(result.command.vx - 0.25) < 1e-9);
}

void TestHurdleFeedbackSequenceStopsRlAndWaitsForDone() {
  HurdleConfig cfg = vision_core::LoadDefaultAlgorithmConfig().hurdle;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_trigger_window = 1;
  cfg.tilt_trigger_min_hits = 1;
  cfg.rl_stop_duration_sec = 0.50;
  HurdleController controller(cfg);
  const auto acquire_hurdle = Target(0.50, 0.70, 0.20);
  const auto hurdle = Target(0.50, 0.90, 0.20);
  const ActionExecutionFeedback waiting{true, false};
  const ActionExecutionFeedback done{true, true};

  auto result = controller.Compute(acquire_hurdle, 100, 100, 0.0, 0.8, true,
                                   Forward(), waiting);
  assert(result.mode == HurdleMode::kApproach);
  result = controller.Compute(hurdle, 100, 100, 0.02, 0.8, true,
                              Forward(), waiting);
  assert(result.mode == HurdleMode::kTiltCameraDownAndSlow);
  result = controller.Compute(hurdle, 100, 100, 0.1, 0.8, true, Down(),
                              waiting);
  assert(result.mode == HurdleMode::kRlStopping);
  assert(result.command.vx == 0.0);
  result = controller.Compute(hurdle, 100, 100, 0.59, 0.8, true, Down(),
                              waiting);
  assert(result.mode == HurdleMode::kRlStopping);
  result = controller.Compute(hurdle, 100, 100, 0.60, 0.8, true, Down(),
                              waiting);
  assert(result.mode == HurdleMode::kContactWalk);
  assert(result.action_request == HurdleActionRequest::kContactWalk);
  assert(std::abs(result.command.vx - cfg.contact_walk_placeholder_vx) <
         1e-9);

  result = controller.Compute(hurdle, 100, 100, 0.70, 0.8, true, Down(),
                              done);
  assert(result.mode == HurdleMode::kCross);
  assert(result.action_request == HurdleActionRequest::kCross);
  result = controller.Compute(hurdle, 100, 100, 0.80, 0.8, true, Down(),
                              done);
  assert(result.mode == HurdleMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);
}

void TestDefaultEntryRequiresTwentyOfThirtyCloseDetections() {
  HurdleController controller;
  const auto hurdle = Target(0.50, 0.70, 0.20);

  HurdleResult result;
  for (int i = 0; i < 19; ++i) {
    result = controller.Compute(hurdle, 100, 100, i * 0.1, Forward());
    assert(result.mode == HurdleMode::kLineFollow);
  }
  for (int i = 0; i < 10; ++i) {
    result = controller.Compute(std::nullopt, 100, 100,
                                1.9 + i * 0.1, Forward());
    assert(result.mode == HurdleMode::kLineFollow);
  }
  result = controller.Compute(hurdle, 100, 100, 2.9, Forward());
  assert(result.mode == HurdleMode::kApproach);
}

void TestHurdleLossUsesTimedRecoveryAndFails() {
  HurdleConfig cfg = vision_core::LoadDefaultAlgorithmConfig().hurdle;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.lost_frames = 1;
  cfg.recovery_reacquire_min_hits = 2;
  cfg.recovery_timeout_sec = 5.0;
  HurdleController controller(cfg);
  const auto hurdle = Target(0.60, 0.70, 0.20);

  auto result = controller.Compute(hurdle, 100, 100, 0.0, Forward());
  assert(result.mode == HurdleMode::kApproach);
  result = controller.Compute(std::nullopt, 100, 100, 0.1, Forward());
  assert(result.mode == HurdleMode::kRecoveryForward);
  assert(result.active);
  assert(std::abs(result.command.vx - cfg.recovery_forward_vx) < 1e-9);
  result = controller.Compute(hurdle, 100, 100, 0.2, Forward());
  assert(result.mode == HurdleMode::kRecoveryForward);
  result = controller.Compute(hurdle, 100, 100, 0.3, Forward());
  assert(result.mode == HurdleMode::kApproach);

  HurdleController persistent_controller(cfg);
  persistent_controller.Compute(hurdle, 100, 100, 0.0, Forward());
  persistent_controller.Compute(std::nullopt, 100, 100, 0.1, Forward());
  result = persistent_controller.Compute(
      std::nullopt, 100, 100, 5.2, Forward());
  assert(result.mode == HurdleMode::kFailed);
  assert(result.active);
  assert(result.command.vx == 0.0);
  assert(result.command.vy == 0.0);
  assert(result.command.wz == 0.0);

  result = persistent_controller.Compute(hurdle, 100, 100, 5.3, Forward());
  assert(result.mode == HurdleMode::kFailed);
  assert(result.active);
}

} // namespace

int main() {
  TestHurdlePlaceholderSequence();
  TestHurdleUsesBallApproachNumbers();
  TestHurdleFeedbackSequenceStopsRlAndWaitsForDone();
  TestDefaultEntryRequiresTwentyOfThirtyCloseDetections();
  TestHurdleLossUsesTimedRecoveryAndFails();
  std::cout << "hurdle controller tests passed\n";
  return 0;
}
