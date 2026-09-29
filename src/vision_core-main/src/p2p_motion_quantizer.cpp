#include "vision_core/p2p_motion_quantizer.hpp"

#include <algorithm>
#include <cmath>

#include "vision_core/ball_controller.hpp"
#include "vision_core/config_loader.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"

namespace vision_core {
namespace {
const ControlCommandConfig &SharedCommandConfig() {
  static const ControlCommandConfig config =
      LoadDefaultAlgorithmConfig().command;
  return config;
}
} // namespace

P2pMotionProfile SelectP2pMotionProfile(MissionType mission,
                                        int mission_phase) {
  switch (mission) {
  case MissionType::kBall: {
    const auto mode = static_cast<BallMode>(mission_phase);
    if (mode == BallMode::kTiltCameraDownAndApproach ||
        mode == BallMode::kFineAdjustForPickup) {
      return P2pMotionProfile::kFine;
    }
    if (mode == BallMode::kBallRecoveryForward ||
        mode == BallMode::kBallRecoveryDown ||
        mode == BallMode::kPostPickupBackAway ||
        mode == BallMode::kPostPickupLineRecovery) {
      return P2pMotionProfile::kRecovery;
    }
    break;
  }
  case MissionType::kHurdle: {
    const auto mode = static_cast<HurdleMode>(mission_phase);
    if (mode == HurdleMode::kTiltCameraDownAndSlow) {
      return P2pMotionProfile::kFine;
    }
    if (mode == HurdleMode::kRecoveryForward ||
        mode == HurdleMode::kRecoveryDown || mode == HurdleMode::kFailed) {
      return P2pMotionProfile::kRecovery;
    }
    break;
  }
  case MissionType::kGoal: {
    const auto mode = static_cast<GoalMode>(mission_phase);
    if (mode == GoalMode::kApproach || mode == GoalMode::kFineAdjust) {
      return P2pMotionProfile::kFine;
    }
    if (mode == GoalMode::kSearch || mode == GoalMode::kHeadingRecovery) {
      return P2pMotionProfile::kRecovery;
    }
    break;
  }
  default:
    break;
  }
  return P2pMotionProfile::kNormal;
}

P2pMotionQuantizer::P2pMotionQuantizer()
    : P2pMotionQuantizer(SharedCommandConfig().p2p,
                         SharedCommandConfig().p2p_fine,
                         SharedCommandConfig().p2p_recovery) {}

P2pMotionQuantizer::P2pMotionQuantizer(const P2pMotionConfig &config)
    : P2pMotionQuantizer(config, SharedCommandConfig().p2p_fine,
                         SharedCommandConfig().p2p_recovery) {}

P2pMotionQuantizer::P2pMotionQuantizer(const P2pMotionConfig &normal_config,
                                       const P2pMotionConfig &fine_config,
                                       const P2pMotionConfig &recovery_config)
    : normal_config_(normal_config), fine_config_(fine_config),
      recovery_config_(recovery_config) {}

LocomotionAction
P2pMotionQuantizer::Quantize(const MotionCommand &command) const {
  return QuantizeWithConfig(command, normal_config_);
}

LocomotionAction P2pMotionQuantizer::Quantize(const MotionCommand &command,
                                              MissionType mission,
                                              int mission_phase) const {
  if (mission == MissionType::kLine) {
    return QuantizeLineWithConfig(command, normal_config_);
  }
  switch (SelectP2pMotionProfile(mission, mission_phase)) {
  case P2pMotionProfile::kFine:
    return QuantizeWithConfig(command, fine_config_);
  case P2pMotionProfile::kRecovery:
    return QuantizeWithConfig(command, recovery_config_);
  case P2pMotionProfile::kNormal:
  default:
    return QuantizeWithConfig(command, normal_config_);
  }
}

LocomotionAction P2pMotionQuantizer::QuantizeLineWithConfig(
    const MotionCommand &command, const P2pMotionConfig &config) const {
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
      !std::isfinite(command.wz)) {
    return LocomotionAction::kNone;
  }

  const double forward_deadband = std::max(0.0, config.forward_deadband);
  const double lateral_deadband = std::max(0.0, config.lateral_deadband);
  const double yaw_deadband = std::max(0.0, config.yaw_deadband);
  // LOCAL TUNING OVERRIDE
  // 기본은 config(YAML/ROS override)를 사용한다. 빠른 재빌드 실험 때만
  // 원하는 우변을 숫자 literal로 바꾸고, 확정 뒤에는 config로 복구한다.
  const double long_forward_vx =
      std::max(forward_deadband, config.long_forward_vx);
  const double curve_yaw_threshold =
      std::max(yaw_deadband, config.curve_yaw_threshold);
  const double sharp_turn_yaw_threshold =
      std::max(curve_yaw_threshold, config.sharp_turn_yaw_threshold);
  const double vx = std::abs(command.vx) >= forward_deadband ? command.vx : 0.0;
  const double vy = std::abs(command.vy) >= lateral_deadband ? command.vy : 0.0;
  const double wz = std::abs(command.wz) >= yaw_deadband ? command.wz : 0.0;

  if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
    return LocomotionAction::kNone;
  }
  if (vx < 0.0) return LocomotionAction::kStepBack;
  if (vx > 0.0) {
    const bool long_walk = vx >= long_forward_vx;
    const double abs_wz = std::abs(wz);
    if (!long_walk) {
      if (abs_wz >= curve_yaw_threshold) {
        return wz > 0.0 ? LocomotionAction::kTurnLeft
                        : LocomotionAction::kTurnRight;
      }
      return LocomotionAction::kStepForwardOne;
    }
    if (abs_wz >= sharp_turn_yaw_threshold) {
      return wz > 0.0 ? LocomotionAction::kTurnLeftAndStep
                      : LocomotionAction::kTurnRightAndStep;
    }
    if (abs_wz >= curve_yaw_threshold) {
      return wz > 0.0 ? LocomotionAction::kStepForwardLeft
                      : LocomotionAction::kStepForwardRight;
    }
    return LocomotionAction::kStepForwardFive;
  }
  if (vy != 0.0) {
    return vy > 0.0 ? LocomotionAction::kLeftSideStep
                    : LocomotionAction::kRightSideStep;
  }
  return wz > 0.0 ? LocomotionAction::kTurnLeft
                  : LocomotionAction::kTurnRight;
}

LocomotionAction
P2pMotionQuantizer::QuantizeWithConfig(const MotionCommand &command,
                                       const P2pMotionConfig &config) const {
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
      !std::isfinite(command.wz)) {
    return LocomotionAction::kNone;
  }

  const double forward_deadband = std::max(0.0, config.forward_deadband);
  const double lateral_deadband = std::max(0.0, config.lateral_deadband);
  const double yaw_deadband = std::max(0.0, config.yaw_deadband);
  const double vx = std::abs(command.vx) >= forward_deadband ? command.vx : 0.0;
  const double vy = std::abs(command.vy) >= lateral_deadband ? command.vy : 0.0;
  const double wz = std::abs(command.wz) >= yaw_deadband ? command.wz : 0.0;

  if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
    return LocomotionAction::kNone;
  }

  const double turn_vx_max = std::max(0.0, config.turn_in_place_vx_max);
  if (wz != 0.0 && std::abs(vx) <= turn_vx_max && vy == 0.0) {
    return wz > 0.0 ? LocomotionAction::kTurnLeft
                    : LocomotionAction::kTurnRight;
  }

  const double lateral_ratio = std::max(0.0, config.lateral_dominance_ratio);
  if (vy != 0.0 && std::abs(vy) > std::abs(vx) * lateral_ratio) {
    return vy > 0.0 ? LocomotionAction::kLeftSideStep
                    : LocomotionAction::kRightSideStep;
  }

  if (vx < 0.0)
    return LocomotionAction::kStepBack;

  if (vx > 0.0) {
    const bool long_walk =
        vx >= std::max(forward_deadband, config.long_forward_vx);
    const bool curved =
        std::abs(wz) >= std::max(yaw_deadband, config.curve_yaw_threshold);
    if (curved) {
      if (wz > 0.0) {
        return long_walk ? LocomotionAction::kTurnLeftAndStep
                         : LocomotionAction::kStepForwardLeft;
      }
      return long_walk ? LocomotionAction::kTurnRightAndStep
                       : LocomotionAction::kStepForwardRight;
    }
    return long_walk ? LocomotionAction::kStepForwardFive
                     : LocomotionAction::kStepForwardOne;
  }

  if (vy != 0.0) {
    return vy > 0.0 ? LocomotionAction::kLeftSideStep
                    : LocomotionAction::kRightSideStep;
  }
  return wz > 0.0 ? LocomotionAction::kTurnLeft
                  : LocomotionAction::kTurnRight;
}

MissionAction ToActionCode(LocomotionAction action) {
  return static_cast<MissionAction>(static_cast<std::uint16_t>(action));
}

} // namespace vision_core
