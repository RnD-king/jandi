#include <cassert>
#include <limits>

#include "vision_core/ball_controller.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/p2p_motion_quantizer.hpp"

int main() {
  using namespace vision_core;

  static_assert(static_cast<std::uint16_t>(MissionAction::kTurnLeft) == 7);
  static_assert(static_cast<std::uint16_t>(MissionAction::kTurnRight) == 8);
  static_assert(static_cast<std::uint16_t>(MissionAction::kStepForwardOne) ==
                10);
  static_assert(static_cast<std::uint16_t>(MissionAction::kStepForwardLeft) ==
                11);
  static_assert(static_cast<std::uint16_t>(MissionAction::kStepForwardRight) ==
                12);
  static_assert(static_cast<std::uint16_t>(MissionAction::kStepForwardFive) ==
                13);
  static_assert(static_cast<std::uint16_t>(MissionAction::kTurnLeftAndStep) ==
                18);
  static_assert(static_cast<std::uint16_t>(MissionAction::kTurnRightAndStep) ==
                19);

  P2pMotionQuantizer quantizer;
  assert(quantizer.Quantize({}) == LocomotionAction::kNone);
  assert(quantizer.Quantize({0.10, 0.0, 0.0}) ==
         LocomotionAction::kStepForwardOne);
  assert(quantizer.Quantize({0.30, 0.0, 0.0}) ==
         LocomotionAction::kStepForwardFive);
  assert(quantizer.Quantize({0.30, 0.0, 0.20}) ==
         LocomotionAction::kTurnLeftAndStep);
  assert(quantizer.Quantize({0.30, 0.0, -0.20}) ==
         LocomotionAction::kTurnRightAndStep);
  assert(quantizer.Quantize({0.10, 0.0, 0.20}) ==
         LocomotionAction::kStepForwardLeft);
  assert(quantizer.Quantize({-0.10, 0.0, 0.0}) ==
         LocomotionAction::kStepBack);
  assert(quantizer.Quantize({0.02, 0.10, 0.0}) ==
         LocomotionAction::kLeftSideStep);
  assert(quantizer.Quantize({0.02, -0.10, 0.0}) ==
         LocomotionAction::kRightSideStep);
  assert(quantizer.Quantize({0.0, 0.0, 0.20}) ==
         LocomotionAction::kTurnLeft);
  assert(quantizer.Quantize({0.0, 0.0, -0.20}) ==
         LocomotionAction::kTurnRight);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  assert(quantizer.Quantize({nan, 0.0, 0.0}) ==
         LocomotionAction::kNone);
  assert(ToActionCode(LocomotionAction::kStepForwardFive) ==
         MissionAction::kStepForwardFive);

  assert(SelectP2pMotionProfile(
             MissionType::kLine, 0) == P2pMotionProfile::kNormal);
  assert(SelectP2pMotionProfile(
             MissionType::kBall,
             static_cast<int>(BallMode::kTiltCameraDownAndApproach)) ==
         P2pMotionProfile::kFine);
  assert(SelectP2pMotionProfile(
             MissionType::kBall,
             static_cast<int>(BallMode::kPostPickupLineRecovery)) ==
         P2pMotionProfile::kRecovery);
  assert(SelectP2pMotionProfile(
             MissionType::kHurdle,
             static_cast<int>(HurdleMode::kTiltCameraDownAndSlow)) ==
         P2pMotionProfile::kFine);
  assert(SelectP2pMotionProfile(
             MissionType::kGoal,
             static_cast<int>(GoalMode::kSearch)) ==
         P2pMotionProfile::kRecovery);

  // 같은 속도라도 Normal은 5걸음, Fine/Recovery는 1걸음으로 짧게
  // 재관측한다.
  assert(quantizer.Quantize({0.30, 0.0, 0.0}, MissionType::kLine, 0) ==
         LocomotionAction::kStepForwardFive);
  assert(quantizer.Quantize({}, MissionType::kLine, 0) ==
         LocomotionAction::kNone);
  assert(quantizer.Quantize({0.30, 0.0, 0.20}, MissionType::kLine, 0) ==
         LocomotionAction::kStepForwardLeft);
  assert(quantizer.Quantize({0.30, 0.0, -0.20}, MissionType::kLine, 0) ==
         LocomotionAction::kStepForwardRight);
  assert(quantizer.Quantize({0.30, 0.0, 0.30}, MissionType::kLine, 0) ==
         LocomotionAction::kTurnLeftAndStep);
  assert(quantizer.Quantize({0.30, 0.0, -0.30}, MissionType::kLine, 0) ==
         LocomotionAction::kTurnRightAndStep);
  assert(quantizer.Quantize({0.10, 0.0, 0.05}, MissionType::kLine, 0) ==
         LocomotionAction::kStepForwardOne);
  assert(quantizer.Quantize({0.10, 0.0, 0.10}, MissionType::kLine, 0) ==
         LocomotionAction::kTurnLeft);
  assert(quantizer.Quantize({0.10, 0.0, -0.10}, MissionType::kLine, 0) ==
         LocomotionAction::kTurnRight);
  assert(quantizer.Quantize(
             {0.30, 0.0, 0.0}, MissionType::kBall,
             static_cast<int>(BallMode::kTiltCameraDownAndApproach)) ==
         LocomotionAction::kStepForwardOne);
  assert(quantizer.Quantize(
             {0.30, 0.0, 0.0}, MissionType::kGoal,
             static_cast<int>(GoalMode::kSearch)) ==
         LocomotionAction::kStepForwardOne);
  return 0;
}
