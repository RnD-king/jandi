#pragma once

#include <cstdint>

#include "vision_core/types.hpp"

namespace vision_core {

// P2P 실행기가 제공하는 고정 보행 primitive다. 숫자는 외부
// ActionCommand의 단일 action 규약과 일치한다.
enum class LocomotionAction : std::uint16_t {
  kNone = 0,
  kStepBack = static_cast<std::uint16_t>(MissionAction::kStepBack),
  kLeftSideStep =
      static_cast<std::uint16_t>(MissionAction::kLeftSideStep),
  kRightSideStep =
      static_cast<std::uint16_t>(MissionAction::kRightSideStep),
  kTurnLeft = static_cast<std::uint16_t>(MissionAction::kTurnLeft),
  kTurnRight = static_cast<std::uint16_t>(MissionAction::kTurnRight),
  kStepForwardOne =
      static_cast<std::uint16_t>(MissionAction::kStepForwardOne),
  kStepForwardLeft =
      static_cast<std::uint16_t>(MissionAction::kStepForwardLeft),
  kStepForwardRight =
      static_cast<std::uint16_t>(MissionAction::kStepForwardRight),
  kStepForwardFive =
      static_cast<std::uint16_t>(MissionAction::kStepForwardFive),
  kTurnLeftAndStep =
      static_cast<std::uint16_t>(MissionAction::kTurnLeftAndStep),
  kTurnRightAndStep =
      static_cast<std::uint16_t>(MissionAction::kTurnRightAndStep),
};

struct P2pMotionConfig {
  // 이 값보다 작은 명령축은 0으로 취급한다.
  double forward_deadband{};
  double lateral_deadband{};
  double yaw_deadband{};

  // 전진속도가 이 값 이상이면 1걸음 대신 5걸음 primitive를 선택한다.
  double long_forward_vx{};
  // 라인 긴 전진에서는 yaw가 이 값 이상이면 곡선 보행을,
  // 저속 전진에서는 제자리회전을 선택한다.
  double curve_yaw_threshold{};
  // 라인 긴 전진에서 yaw가 이 값 이상이면 곡선 보행 대신
  // 제자리회전 반복 + 직진 복합 primitive를 선택한다.
  double sharp_turn_yaw_threshold{};
  // 전후진이 이 값 이하인 회전 명령은 제자리 회전으로 취급한다.
  double turn_in_place_vx_max{};
  // |vy|가 |vx|의 이 배수보다 크면 전진보다 횡이동을 우선한다.
  double lateral_dominance_ratio{};
};

enum class P2pMotionProfile {
  kNormal = 0,
  kFine = 1,
  kRecovery = 2,
};

// 같은 phase 숫자라도 미션마다 의미가 다르므로 반드시 둘을 함께 본다.
P2pMotionProfile SelectP2pMotionProfile(MissionType mission,
                                        int mission_phase);

// 연속 속도 의도를 하나의 고정 보행 primitive로 양자화한다. 상태를 보존하지
// 않으므로 각 primitive DONE 뒤 최신 속도로 다시 호출할 수 있다.
class P2pMotionQuantizer {
public:
  P2pMotionQuantizer();
  explicit P2pMotionQuantizer(const P2pMotionConfig &config);
  P2pMotionQuantizer(const P2pMotionConfig &normal_config,
                     const P2pMotionConfig &fine_config,
                     const P2pMotionConfig &recovery_config);

  LocomotionAction Quantize(const MotionCommand &command) const;
  LocomotionAction Quantize(const MotionCommand &command,
                            MissionType mission, int mission_phase) const;

private:
  LocomotionAction QuantizeLineWithConfig(
      const MotionCommand &command, const P2pMotionConfig &config) const;
  LocomotionAction QuantizeWithConfig(const MotionCommand &command,
                                      const P2pMotionConfig &config) const;

  P2pMotionConfig normal_config_;
  P2pMotionConfig fine_config_;
  P2pMotionConfig recovery_config_;
};

// ControlCommand.action은 기존 ROS/C++ 호환 때문에 MissionAction 타입을 유지한다.
// P2P 보행 코드는 동일한 숫자 영역으로 안전하게 변환한다.
MissionAction ToActionCode(LocomotionAction action);

} // namespace vision_core
