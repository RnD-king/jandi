#include "vision_core/control_command.hpp"

#include "vision_core/config_loader.hpp"

#include <cmath>
#include <limits>

namespace vision_core {
namespace {

MissionAction GoalFineAction(const MotionCommand &command) {
  if (std::abs(command.wz) > 1e-9) {
    return command.wz > 0.0 ? MissionAction::kTurnLeft
                            : MissionAction::kTurnRight;
  }
  if (std::abs(command.vy) > 1e-9) {
    return command.vy > 0.0 ? MissionAction::kLeftSideStep
                            : MissionAction::kRightSideStep;
  }
  if (std::abs(command.vx) > 1e-9) {
    return command.vx > 0.0 ? MissionAction::kStepForwardHalf
                            : MissionAction::kStepBack;
  }
  return MissionAction::kNone;
}

MissionAction RequestedAction(MissionType mission, const BallResult &ball,
                              const HurdleResult &hurdle,
                              const GoalResult &goal) {
  if (mission == MissionType::kGoal) {
    if (goal.action_request == GoalActionRequest::kShoot) {
      return MissionAction::kShoot;
    }
    if (goal.action_request == GoalActionRequest::kFineAdjust) {
      return GoalFineAction(goal.command);
    }
    if (goal.action_request == GoalActionRequest::kFineAdjustHold) {
      return MissionAction::kDefaultPoseMode;
    }
    return MissionAction::kNone;
  }
  if (mission == MissionType::kBall) {
    switch (ball.action_request) {
    case BallActionRequest::kFineAdjustForward:
      return MissionAction::kStepForwardHalf;
    case BallActionRequest::kPickup:
      return MissionAction::kPickBall;
    case BallActionRequest::kStandUp:
      return MissionAction::kDefaultPosition;
    case BallActionRequest::kVerifyPickup:
      return MissionAction::kRecatch;
    case BallActionRequest::kNone:
      return MissionAction::kNone;
    }
  }
  if (mission != MissionType::kHurdle) return MissionAction::kNone;
  switch (hurdle.action_request) {
  case HurdleActionRequest::kContactWalk:
    // 접촉 위치까지의 짧은 전진과 실제 허들 넘기를 서로 다른 action으로
    // 유지해야 DONE 뒤 다음 단계가 억제되지 않는다.
    return MissionAction::kStepForwardOne;
  case HurdleActionRequest::kCross:
    return MissionAction::kHurdle;
  case HurdleActionRequest::kNone:
    break;
  }
  return MissionAction::kNone;
}

ActionExecutionKind ExecutionKindForMissionAction(MissionAction action) {
  switch (action) {
  case MissionAction::kStepForwardHalf:
  case MissionAction::kStepBack:
  case MissionAction::kLeftSideStep:
  case MissionAction::kRightSideStep:
  case MissionAction::kTurnLeft:
  case MissionAction::kTurnRight:
  case MissionAction::kStepForwardOne:
  case MissionAction::kStepForwardLeft:
  case MissionAction::kStepForwardRight:
  case MissionAction::kStepForwardFive:
  case MissionAction::kTurnLeftAndStep:
  case MissionAction::kTurnRightAndStep:
    return ActionExecutionKind::kVelocityCompatible;
  case MissionAction::kDefaultPoseMode:
    return ActionExecutionKind::kStationary;
  case MissionAction::kDefaultPosition:
  case MissionAction::kWalkMode:
  case MissionAction::kPickBall:
  case MissionAction::kHurdle:
  case MissionAction::kShoot:
  case MissionAction::kRecatch:
    return ActionExecutionKind::kDiscrete;
  case MissionAction::kNone:
    break;
  }
  return ActionExecutionKind::kNone;
}

MissionType AcquireMission(const BallResult &ball, const HurdleResult &hurdle,
                           const GoalResult &goal) {
  if (goal.mode != GoalMode::kLineFollow) return MissionType::kGoal;
  if (ball.mode == BallMode::kPostPickupLineRecovery) {
    return MissionType::kBall;
  }
  if (!ball.has_ball && hurdle.active) return MissionType::kHurdle;
  if (ball.active) return MissionType::kBall;
  return MissionType::kLine;
}

bool MissionFinished(MissionType mission, const BallResult &ball,
                     const HurdleResult &hurdle, const GoalResult &goal) {
  switch (mission) {
  case MissionType::kBall:
    return ball.mode == BallMode::kLineFollow && !ball.active;
  case MissionType::kGoal:
    return goal.mode == GoalMode::kLineFollow && !goal.active;
  case MissionType::kHurdle:
    return hurdle.mode == HurdleMode::kLineFollow && !hurdle.active;
  default:
    return true;
  }
}

MotionCommand CommandForMission(MissionType mission, const BallResult &ball,
                                const HurdleResult &hurdle,
                                const GoalResult &goal,
                                const MotionCommand &line_candidate) {
  switch (mission) {
  case MissionType::kBall:
    if (ball.mode == BallMode::kPostPickupLineRecovery) {
      MotionCommand command = line_candidate;
      command.vx = 0.0;
      command.vy = 0.0;
      return command;
    }
    return ball.active ? ball.command : MotionCommand{};
  case MissionType::kGoal:
    return goal.active ? goal.command : line_candidate;
  case MissionType::kHurdle:
    return hurdle.active ? hurdle.command : MotionCommand{};
  default:
    return line_candidate;
  }
}

int MissionPhase(MissionType mission, const BallResult &ball,
                 const HurdleResult &hurdle, const GoalResult &goal) {
  switch (mission) {
  case MissionType::kBall: return static_cast<int>(ball.mode);
  case MissionType::kGoal: return static_cast<int>(goal.mode);
  case MissionType::kHurdle: return static_cast<int>(hurdle.mode);
  default: return 0;
  }
}

CameraRequest CameraForMission(MissionType mission, const BallResult &ball,
                               const HurdleResult &hurdle,
                               const GoalResult &goal) {
  switch (mission) {
  case MissionType::kBall: return ball.camera_request;
  case MissionType::kGoal: return goal.camera_request;
  case MissionType::kHurdle: return hurdle.camera_request;
  default: return CameraRequest::kNone;
  }
}

bool IsRlStopping(MissionType mission, const BallResult &ball,
                  const HurdleResult &hurdle, const GoalResult &goal) {
  return (mission == MissionType::kBall &&
          ball.mode == BallMode::kRlStoppingForPickup) ||
         (mission == MissionType::kGoal &&
          goal.mode == GoalMode::kRlStopping) ||
         (mission == MissionType::kHurdle &&
          hurdle.mode == HurdleMode::kRlStopping);
}

bool IsLongLineLocomotionAction(MissionAction action) {
  return action == MissionAction::kStepForwardFive ||
         action == MissionAction::kStepForwardLeft ||
         action == MissionAction::kStepForwardRight ||
         action == MissionAction::kTurnLeftAndStep ||
         action == MissionAction::kTurnRightAndStep;
}

ActionExecutionKind ExecutionKindForLocomotionAction(MissionAction action) {
  return action == MissionAction::kNone
             ? ActionExecutionKind::kNone
             : ActionExecutionKind::kVelocityCompatible;
}

double LocomotionTargetYawRad(MissionAction action,
                              const MotionCommand &command) {
  if (!std::isfinite(command.wz)) return 0.0;
  if (action == MissionAction::kTurnLeft ||
      action == MissionAction::kTurnRight ||
      action == MissionAction::kTurnLeftAndStep ||
      action == MissionAction::kTurnRightAndStep) {
    // 방향은 action 코드가 구분하므로 라인 회전 목표각은 항상
    // 양수 크기로 전달한다. SHOOT의 signed yaw와는 별도 규약이다.
    return std::abs(command.wz);
  }
  return 0.0;
}

} // namespace

ControlCommandCoordinator::ControlCommandCoordinator()
    : ControlCommandCoordinator(LoadDefaultAlgorithmConfig().command) {}

ControlCommandCoordinator::ControlCommandCoordinator(
    const ControlCommandConfig &config)
    : config_(config),
      p2p_quantizer_(config.p2p, config.p2p_fine, config.p2p_recovery),
      next_action_id_(config.first_action_id == 0 ? 1
                                                  : config.first_action_id) {}

ControlCommand ControlCommandCoordinator::BeginAction(
    ControlCommand command, MissionAction action, ActionCategory category,
    ActionExecutionKind execution_kind, double action_yaw_rad) {
  pending_action_id_ = next_action_id_++;
  // 0은 외부 프로토콜에서 "pending 없음"이므로 wrap-around해도 건너뛴다.
  if (next_action_id_ == 0) next_action_id_ = 1;
  pending_action_ = action;
  pending_action_category_ = category;
  pending_action_execution_kind_ = execution_kind;
  pending_pre_p2p_motion_ = command.pre_p2p_motion;
  pending_action_yaw_rad_ =
      std::isfinite(action_yaw_rad) ? action_yaw_rad : 0.0;
  pending_action_issued_sec_ = compute_time_valid_ ? compute_now_sec_ : 0.0;
  pending_acknowledged_ = false;
  pending_ready_ = false;
  command.command_type = CommandType::kAction;
  command.control_phase = ControlPhase::kWaitingActionAck;
  command.velocity = {};
  command.action = pending_action_;
  command.action_category = pending_action_category_;
  command.action_execution_kind = pending_action_execution_kind_;
  command.action_id = pending_action_id_;
  command.action_yaw_rad = pending_action_yaw_rad_;
  return command;
}

ControlCommand ControlCommandCoordinator::BeginQueuedLineAction(
    ControlCommand command, MissionAction action) {
  queued_action_id_ = next_action_id_++;
  if (next_action_id_ == 0) next_action_id_ = 1;
  queued_action_ = action;
  queued_pre_p2p_motion_ = command.pre_p2p_motion;
  queued_action_yaw_rad_ =
      LocomotionTargetYawRad(action, queued_pre_p2p_motion_);
  queued_action_issued_sec_ = compute_time_valid_ ? compute_now_sec_ : 0.0;
  queued_acknowledged_ = false;
  command.command_type = CommandType::kAction;
  command.control_phase = ControlPhase::kWaitingQueuedActionAck;
  command.velocity = {};
  command.action = queued_action_;
  command.action_category = ActionCategory::kLocomotion;
  command.action_execution_kind = ActionExecutionKind::kVelocityCompatible;
  command.action_id = queued_action_id_;
  command.action_yaw_rad = queued_action_yaw_rad_;
  return command;
}

ControlCommand ControlCommandCoordinator::Compute(
    const BallResult &ball_result, const HurdleResult &hurdle_result,
    const GoalResult &goal_result, const MotionCommand &line_candidate,
    const CommandDeliveryFeedback &feedback) {
  return Compute(ball_result, hurdle_result, goal_result, line_candidate,
                 feedback, std::numeric_limits<double>::quiet_NaN());
}

ControlCommand ControlCommandCoordinator::Compute(
    const BallResult &ball_result, const HurdleResult &hurdle_result,
    const GoalResult &goal_result, const MotionCommand &line_candidate,
    const CommandDeliveryFeedback &feedback, double now_sec,
    bool allow_new_line_locomotion_action) {
  compute_time_valid_ = std::isfinite(now_sec);
  compute_now_sec_ = compute_time_valid_ ? now_sec : 0.0;

  if (!action_ack_timed_out_ && queued_action_id_ != 0 &&
      feedback.action_id == queued_action_id_) {
    queued_acknowledged_ = queued_acknowledged_ || feedback.acknowledged;
  }
  if (!action_ack_timed_out_ && pending_action_id_ != 0 &&
      feedback.action_id == pending_action_id_) {
    pending_acknowledged_ = pending_acknowledged_ || feedback.acknowledged;
    pending_ready_ = pending_ready_ || feedback.ready;
    if (feedback.done) {
      if (pending_action_category_ == ActionCategory::kMission) {
        // DEFAULT_POSE_MODE는 정지 상태에서 관측을 더 모으기 위해 연속 실행될
        // 수 있는 반복 측정 액션이다. DONE 뒤 같은 HOLD가 다시 필요하면 새
        // ID로 허용하고, 나머지 단발 미션 액션만 기존처럼 억제한다.
        suppressed_mission_action_ =
            pending_action_ == MissionAction::kDefaultPoseMode
                ? MissionAction::kNone
                : pending_action_;
      }
      if (queued_action_id_ != 0) {
        pending_action_id_ = queued_action_id_;
        pending_action_ = queued_action_;
        pending_action_category_ = ActionCategory::kLocomotion;
        pending_action_execution_kind_ =
            ActionExecutionKind::kVelocityCompatible;
        pending_pre_p2p_motion_ = queued_pre_p2p_motion_;
        pending_action_yaw_rad_ = queued_action_yaw_rad_;
        pending_action_issued_sec_ = queued_action_issued_sec_;
        pending_acknowledged_ = queued_acknowledged_;
        pending_ready_ = false;
        queued_action_id_ = 0;
        queued_action_ = MissionAction::kNone;
        queued_pre_p2p_motion_ = {};
        queued_action_yaw_rad_ = 0.0;
        queued_action_issued_sec_ = 0.0;
        queued_acknowledged_ = false;
      } else {
        pending_action_id_ = 0;
        pending_action_ = MissionAction::kNone;
        pending_action_category_ = ActionCategory::kNone;
        pending_action_execution_kind_ = ActionExecutionKind::kNone;
        pending_pre_p2p_motion_ = {};
        pending_action_yaw_rad_ = 0.0;
        pending_action_issued_sec_ = 0.0;
        pending_acknowledged_ = false;
        pending_ready_ = false;
      }
    }
  }

  if (!action_ack_timed_out_ && compute_time_valid_) {
    const double timeout_sec = std::max(0.0, config_.action_ack_timeout_sec);
    const auto expired = [&](double issued_sec) {
      return timeout_sec > 0.0 && compute_now_sec_ >= issued_sec &&
             compute_now_sec_ - issued_sec >= timeout_sec;
    };
    const bool pending_timeout = pending_action_id_ != 0 &&
                                 !pending_acknowledged_ &&
                                 expired(pending_action_issued_sec_);
    const bool queued_timeout = queued_action_id_ != 0 &&
                                !queued_acknowledged_ &&
                                expired(queued_action_issued_sec_);
    action_ack_timed_out_ = pending_timeout || queued_timeout;
  }

  // 실행 중인 액션까지 끝난 뒤 controller의 명시적인 종료 상태만 잠금을 푼다.
  if (active_mission_ != MissionType::kLine && pending_action_id_ == 0 &&
      MissionFinished(active_mission_, ball_result, hurdle_result,
                      goal_result)) {
    active_mission_ = MissionType::kLine;
  }
  if (active_mission_ == MissionType::kLine) {
    active_mission_ = AcquireMission(ball_result, hurdle_result, goal_result);
  }

  ControlCommand command;
  command.mission = active_mission_;
  command.mission_phase = MissionPhase(command.mission, ball_result,
                                       hurdle_result, goal_result);
  command.camera_request = CameraForMission(command.mission, ball_result,
                                            hurdle_result, goal_result);
  command.pre_p2p_motion = CommandForMission(
      command.mission, ball_result, hurdle_result, goal_result,
      line_candidate);

  const MissionAction requested = RequestedAction(
      command.mission, ball_result, hurdle_result, goal_result);

  if (action_ack_timed_out_) {
    // 늦은 ACK 뒤 동일 동작을 새 ID로 다시 실행하는 위험을 피하기 위해 자동
    // 재시도하지 않는다. 외부 감독자가 원인을 처리한 뒤 Reset해야 한다.
    const bool queued_failed = queued_action_id_ != 0 && !queued_acknowledged_;
    command.command_type = CommandType::kHold;
    command.control_phase = ControlPhase::kActionAckTimedOut;
    command.velocity = {};
    command.action_id = queued_failed ? queued_action_id_ : pending_action_id_;
    command.action = queued_failed ? queued_action_ : pending_action_;
    command.action_category = queued_failed ? ActionCategory::kLocomotion
                                            : pending_action_category_;
    command.action_execution_kind =
        queued_failed ? ActionExecutionKind::kVelocityCompatible
                      : pending_action_execution_kind_;
    command.pre_p2p_motion = queued_failed ? queued_pre_p2p_motion_
                                           : pending_pre_p2p_motion_;
    command.action_yaw_rad = queued_failed ? queued_action_yaw_rad_
                                           : pending_action_yaw_rad_;
    return command;
  }

  if (pending_action_id_ != 0) {
    if (allow_new_line_locomotion_action && queued_action_id_ == 0 &&
        pending_ready_ &&
        command.mission == MissionType::kLine &&
        pending_action_category_ == ActionCategory::kLocomotion &&
        IsLongLineLocomotionAction(pending_action_)) {
      const LocomotionAction next = p2p_quantizer_.Quantize(
          command.pre_p2p_motion, MissionType::kLine, 0);
      if (next != LocomotionAction::kNone) {
        return BeginQueuedLineAction(command, ToActionCode(next));
      }
    }
    if (queued_action_id_ != 0) {
      command.action_id = queued_action_id_;
      command.action = queued_action_;
      command.action_category = ActionCategory::kLocomotion;
      command.action_execution_kind = ActionExecutionKind::kVelocityCompatible;
      command.pre_p2p_motion = queued_pre_p2p_motion_;
      command.action_yaw_rad = queued_action_yaw_rad_;
      command.velocity = {};
      command.command_type = queued_acknowledged_ ? CommandType::kHold
                                                  : CommandType::kAction;
      command.control_phase =
          queued_acknowledged_ ? ControlPhase::kWaitingQueuedActionStart
                               : ControlPhase::kWaitingQueuedActionAck;
      return command;
    }
    command.action_id = pending_action_id_;
    command.action = pending_action_;
    command.action_category = pending_action_category_;
    command.action_execution_kind = pending_action_execution_kind_;
    command.action_yaw_rad = pending_action_yaw_rad_;
    command.pre_p2p_motion = pending_pre_p2p_motion_;
    command.velocity = {};
    command.command_type = pending_acknowledged_ ? CommandType::kHold
                                                 : CommandType::kAction;
    command.control_phase = pending_acknowledged_
                                ? ControlPhase::kWaitingActionDone
                                : ControlPhase::kWaitingActionAck;
    return command;
  }

  if (requested == MissionAction::kNone) {
    suppressed_mission_action_ = MissionAction::kNone;
  } else if (requested != suppressed_mission_action_) {
    const double action_yaw_rad =
        command.mission == MissionType::kGoal &&
                requested == MissionAction::kShoot
            ? goal_result.shoot_yaw_rad
            : 0.0;
    const ActionExecutionKind execution_kind =
        ExecutionKindForMissionAction(requested);
    if (execution_kind != ActionExecutionKind::kVelocityCompatible) {
      command.pre_p2p_motion = {};
    }
    return BeginAction(command, requested, ActionCategory::kMission,
                       execution_kind, action_yaw_rad);
  }

  command.velocity = command.pre_p2p_motion;
  if (IsRlStopping(command.mission, ball_result, hurdle_result, goal_result)) {
    command.command_type =
        config_.locomotion_backend == LocomotionBackend::kVelocity
            ? CommandType::kVelocity
            : CommandType::kHold;
    command.control_phase = ControlPhase::kRlStopping;
    command.pre_p2p_motion = {};
    command.velocity = {};
  } else if (requested != MissionAction::kNone) {
    command.command_type = CommandType::kHold;
    command.pre_p2p_motion = {};
    command.velocity = {};
  } else if (config_.locomotion_backend ==
             LocomotionBackend::kP2pAction) {
    if (!allow_new_line_locomotion_action &&
        command.mission == MissionType::kLine) {
      command.command_type = CommandType::kHold;
      command.velocity = {};
      command.pre_p2p_motion = {};
      return command;
    }
    const LocomotionAction locomotion =
        p2p_quantizer_.Quantize(command.velocity, command.mission,
                                command.mission_phase);
    command.velocity = {};
    if (locomotion == LocomotionAction::kNone) {
      command.command_type = CommandType::kHold;
    } else {
      return BeginAction(command, ToActionCode(locomotion),
                         ActionCategory::kLocomotion,
                         ExecutionKindForLocomotionAction(
                             ToActionCode(locomotion)),
                         LocomotionTargetYawRad(ToActionCode(locomotion),
                                                command.pre_p2p_motion));
    }
  } else {
    command.command_type = CommandType::kVelocity;
  }
  return command;
}

void ControlCommandCoordinator::UpdateNormalP2pConfig(
    const P2pMotionConfig &config) {
  config_.p2p = config;
  p2p_quantizer_ =
      P2pMotionQuantizer(config_.p2p, config_.p2p_fine,
                         config_.p2p_recovery);
}

void ControlCommandCoordinator::Reset() {
  active_mission_ = MissionType::kLine;
  pending_action_id_ = 0;
  pending_action_ = MissionAction::kNone;
  pending_action_category_ = ActionCategory::kNone;
  pending_action_execution_kind_ = ActionExecutionKind::kNone;
  pending_pre_p2p_motion_ = {};
  pending_action_yaw_rad_ = 0.0;
  pending_action_issued_sec_ = 0.0;
  pending_acknowledged_ = false;
  pending_ready_ = false;
  action_ack_timed_out_ = false;
  queued_action_id_ = 0;
  queued_action_ = MissionAction::kNone;
  queued_pre_p2p_motion_ = {};
  queued_action_yaw_rad_ = 0.0;
  queued_action_issued_sec_ = 0.0;
  queued_acknowledged_ = false;
  compute_now_sec_ = 0.0;
  compute_time_valid_ = false;
  suppressed_mission_action_ = MissionAction::kNone;
}

} // namespace vision_core

/*
ControlCommand 결정 규칙 (ROS 통합 경로)
========================================

이 파일은 각 controller가 이미 판단한 결과를 하나의 ControlCommand로 조합한다.
각 controller가 직접 최종 cmd 배열을 만드는 구조는 아니다.

입력별 역할
-----------
- ball_result / hurdle_result / goal_result
    현재 미션 상태(mode), 미션용 속도(command), 동작 요청(action_request),
    카메라 요청(camera_request)을 제공한다.
- line_candidate
    어떤 물체 미션도 속도 제어권을 갖지 않을 때 사용할 라인 추종 속도다.
- feedback
    현재 전송 중인 action_id에 대한 ACK/READY/DONE 수신 결과다.

최종 ControlCommand 항목
-----------------------
- command_type : VELOCITY(1), ACTION(2), HOLD(3) 중 현재 실행 방식을 나타낸다.
- mission      : LINE(1), BALL(2), GOAL(3), HURDLE(4) 중 현재 판단 상태다.
- mission_phase: 해당 controller의 mode enum 숫자를 그대로 넣는다.
- control_phase: 일반 미션(0), RL 정지 중(1), ACK 대기(2), DONE 대기(3),
    예약 ACK 대기(4), 예약 action 시작 대기(5), ACK timeout 정지(6)다.
- pre_p2p_motion: P2P action으로 양자화하기 전 vx/vy/wz 의도다. pending 중에도
    해당 action_id가 처음 선택됐을 때의 값을 유지한다.
- velocity     : VELOCITY일 때 사용할 vx/vy/wz다. ACTION/HOLD에서는 0/0/0이다.
- action       : ACTION/HOLD에서 실행 또는 유지 중인 공통 wire action 코드다.
- action_category: controller가 요청한 MISSION과 P2P가 만든 LOCOMOTION을
    구분한다. ROS 메시지에 싣지 않아도 송신부가 action_id별로 보존할 수 있다.
- action_execution_kind: PRE-P2P velocity로 대체 가능한 보행, 별도 구현이 필요한
    discrete action, 정지 관측 action을 실행기 어댑터가 구분하는 값이다.
- action_id    : 새 ACTION마다 증가하는 식별자다. 같은 동작의 재전송은 같은 ID다.
- camera_request: 활성 미션 controller가 요청한 NONE/DOWN/FORWARD/GOAL 값이다.

1. 활성 미션 잠금과 속도 후보 선택
----------------------------------
active_mission_이 LINE일 때만 새 미션을 고른다. 최초 진입 우선순위는
GOAL(진입 조건을 이미 통과한 경우) > HURDLE > BALL > LINE이며, 공 보유 중에는
HURDLE을 선택하지 않는다.

BALL/HURDLE/GOAL 중 하나가 선택되면 다른 controller 결과가 active가 되어도
현재 미션을 바꾸지 않는다. 현재 controller가 LINE_FOLLOW와 active=false를 함께
반환하고 대기 중 ACTION도 없을 때만 잠금을 해제한다. HURDLE_FAILED처럼
active=true인 종료 정지 상태는 명시적인 Reset 전까지 계속 HURDLE로 잠긴다.

BALL_POST_PICKUP_LINE_RECOVERY는 BALL 잠금을 유지하면서 line_candidate의 wz만
사용하고, GOAL_HEADING_RECOVERY는 GOAL 잠금을 유지하면서 line_candidate 전체를
사용한다.

mission_phase는 선택된 미션의 mode를 static_cast<int>()한 값이다.
LINE이면 항상 0이다. camera_request도 선택된 미션 결과에서만 가져온다.

2. controller의 action_request -> MissionAction 변환
----------------------------------------------------
- BALL/FINE_ADJUST_FORWARD -> STEP_FORWARD_HALF(3)
- BALL/PICKUP              -> PICK_BALL(14)
- BALL/STAND_UP            -> DEFAULT_POSITION(1)
- BALL/VERIFY_PICKUP       -> RECATCH(15)
- HURDLE/CONTACT_WALK      -> STEP_FORWARD_ONE(10)
- HURDLE/CROSS             -> HUDDLE(16)
- GOAL/SHOOT               -> SHOOT(17)
- GOAL/FINE_ADJUST         -> goal_result.command에서 0이 아닌 축을 읽어 변환
    wz > 0: TURN_LEFT(7),       wz < 0: TURN_RIGHT(8)
    vy > 0: LEFT_SIDE_STEP(5),  vy < 0: RIGHT_SIDE_STEP(6)
    vx > 0: STEP_FORWARD_HALF(3), vx < 0: STEP_BACK(4)
  여러 축이 동시에 0이 아니면 wz, vy, vx 순서로 하나만 선택한다.
- GOAL/FINE_ADJUST_HOLD    -> DEFAULT_POSE_MODE(2)

goal.mode != LINE_FOLLOW인 동안에는 Goal action만 검사한다. 그 외에는 Ball,
Hurdle 순서로 검사한다. 각 controller가 kNone을 요청하면 action=NONE(0)이다.

3. ACTION 전송 생명주기
----------------------
- 새 action_request 발견:
    command_type=ACTION, control_phase=WAITING_ACTION_ACK,
    velocity=0/0/0, 새 action_id 발급
- ACK 전:
    매 Compute()에서 동일한 action/action_id로 ACTION을 다시 출력
- ACK 수신:
    command_type=HOLD, control_phase=WAITING_ACTION_DONE,
    동일한 action/action_id와 velocity=0/0/0 유지
- 긴 라인 보행의 READY 수신:
    그 시점까지 누적한 라인 특징으로 다음 action을 정하고, 새 action_id와
    새 action_id를 가진 예약 ACTION을 출력한다. 실행기는 READY를 보낸 현재
    action이 DONE될 때까지 이 새 ID를 큐에 보관한다.
- DONE 수신:
    예약 action이 있으면 실행 중 action으로 승격하고, 없으면 pending action을
    해제한다. 일반 미션 action은 같은 controller 상태에서
    즉시 재발급하지 않으며 request=NONE 뒤 억제를 해제한다. 반복 관측용
    DEFAULT_POSE_MODE(2)는 예외로 DONE 뒤 같은 요청도 새 ID로 발급한다.

ACK/READY/DONE은 feedback.action_id가 현재 실행 또는 예약 ID와 맞을 때만
반영된다.

일반 Mission action은 DONE 뒤 controller가 request=NONE을 출력할 때까지 같은
action을 억제한다. DEFAULT_POSE_MODE와 P2P locomotion action은 반복 가능하므로
DONE 뒤 같은 요청에도 새 action_id를 발급한다.

P2P backend
-----------
인자 없는 coordinator는 공통 algorithm YAML의 backend 설정을 읽는다.
kP2pAction에서는 명시적인 controller action_request를 먼저 처리한 뒤, 요청이
없을 때만 최종 velocity를 P2pMotionQuantizer로 보낸다. 정지/RL_STOPPING은 새
보행 ACTION을 만들지 않는 HOLD가 되고, 0이 아닌 속도는 LOCOMOTION ACTION이 된다.
실행 중 action은 하나이며 긴 라인 보행에서만 다음 locomotion action 하나를
미리 예약할 수 있다. 미션/보행 동작은 동시에 실행되지 않는다.
ROS는 action을 실행하고, MuJoCo는 kVelocityCompatible일 때 pre_p2p_motion을
사용할 수 있다. 두 실행기 모두 같은 action_id에 ACK/DONE을 반환하므로 FSM은 같다.

4. 상태별 최종 cmd
-----------------
아래 표는 controller가 ActionExecutionFeedback.enabled=true로 동작하고,
아직 다른 pending ACTION이 없는 정상 ROS 경로를 기준으로 한다.

BALL
- LINE_FOLLOW(0)
    VELOCITY, LINE(1), phase=0, line_candidate, NONE, camera=NONE
- APPROACH_BALL(1)
    VELOCITY, BALL(2), phase=1, ball.command, NONE, camera=NONE
- TILT_CAMERA_DOWN_AND_APPROACH(2)
    VELOCITY, BALL(2), phase=2, ball.command, NONE, camera=DOWN(1)
- RL_STOPPING_FOR_PICKUP(10)
    VELOCITY, BALL(2), phase=10, 0/0/0, NONE, camera=NONE,
    control_phase=RL_STOPPING(1)
- FINE_ADJUST_FOR_PICKUP(3)
    ACTION, BALL(2), phase=3, 0/0/0, STEP_FORWARD_HALF(3), camera=NONE
- PICKUP_BALL(4)
    ACTION, BALL(2), phase=4, 0/0/0, PICK_BALL(14), camera=NONE
- VERIFY_PICKUP(5)
    ACTION, BALL(2), phase=5, 0/0/0, RECATCH(15), camera=NONE
- STAND_UP_AFTER_PICKUP(6)
    ACTION, BALL(2), phase=6, 0/0/0, DEFAULT_POSITION(1), camera=NONE
- RETURN_CAMERA_TO_LINE(7)
    VELOCITY, BALL(2), phase=7, 0/0/0, NONE, camera=FORWARD(2)
- BALL_RECOVERY_FORWARD(8)
    VELOCITY, BALL(2), phase=8, ball.command, NONE, camera=NONE
- BALL_RECOVERY_DOWN(9)
    VELOCITY, BALL(2), phase=9, ball.command, NONE, camera=DOWN(1)
HURDLE
- LINE_FOLLOW(0)
    VELOCITY, LINE(1), phase=0, line_candidate, NONE, camera=NONE
- APPROACH(1)
    VELOCITY, HURDLE(4), phase=1, hurdle.command, NONE, camera=NONE
- TILT_CAMERA_DOWN_AND_SLOW(2)
    VELOCITY, HURDLE(4), phase=2, hurdle.command, NONE, camera=DOWN(1)
- RL_STOPPING(6)
    VELOCITY, HURDLE(4), phase=6, 0/0/0, NONE, camera=NONE,
    control_phase=RL_STOPPING(1)
- CONTACT_WALK(3)
    ACTION, HURDLE(4), phase=3, 0/0/0, STEP_FORWARD_ONE(10), camera=NONE
- CROSS(4)
    ACTION, HURDLE(4), phase=4, 0/0/0, HUDDLE(16), camera=NONE
- RETURN_CAMERA_TO_LINE(5)
    VELOCITY, HURDLE(4), phase=5, 0/0/0, NONE, camera=FORWARD(2)
GOAL
- LINE_FOLLOW(0)
    VELOCITY, LINE(1), phase=0, line_candidate, NONE, camera=NONE
- POST_PICKUP_WAIT(1)
    VELOCITY, GOAL(3), phase=1, 0/0/0, NONE, camera=NONE
- TILT_CAMERA_TO_GOAL(2)
    VELOCITY, GOAL(3), phase=2, goal.command, NONE, camera=GOAL(3)
- SEARCH(3)
    VELOCITY, GOAL(3), phase=3, goal.command, NONE, camera=NONE
- APPROACH(4)
    VELOCITY, GOAL(3), phase=4, goal.command, NONE, camera=NONE
- RL_STOPPING(9)
    VELOCITY, GOAL(3), phase=9, 0/0/0, NONE, camera=NONE,
    control_phase=RL_STOPPING(1)
- FINE_ADJUST(5), 미세 동작 요청이 있을 때
    ACTION, GOAL(3), phase=5, 0/0/0, 방향에 맞는 STEP/TURN, camera=NONE
- FINE_ADJUST(5), settle/재측정 구간
    ACTION, GOAL(3), phase=5, 0/0/0, DEFAULT_POSE_MODE(2), camera=NONE
- SHOOT(6)
    ACTION, GOAL(3), phase=6, 0/0/0, SHOOT(17), camera=NONE
- RETURN_CAMERA_TO_LINE(7)
    VELOCITY, GOAL(3), phase=7, 0/0/0, NONE, camera=FORWARD(2)
- HEADING_RECOVERY(8)
    VELOCITY, GOAL(3), phase=8, line_candidate, NONE, camera=NONE
pending ACTION 우선 규칙의 뜻
----------------------------
예를 들어 LEFT_SIDE_STEP(id=21)를 한 번 발급했다면, controller가 다음 프레임에 계산한
상태별 기본 cmd보다 id=21의 전달/실행 완료 확인을 먼저 처리한다.

- 아직 ACK가 없으면: ACTION + LEFT_SIDE_STEP + id=21을 다시 출력한다.
- ACK만 있고 DONE이 없으면: HOLD + LEFT_SIDE_STEP + id=21을 출력한다.
  이 HOLD는 새 동작 명령이 아니라 "실행기가 이미 받은 id=21이 끝나기를 기다리는
  중"이라는 내부/통합 cmd 표시다. ROS action_cmd 토픽에는 중복 발행하지 않는다.
- DONE이 오면: id=21을 해제하고 그때 다음 상태의 ACTION을 새 ID로 발급하거나
  VELOCITY 명령으로 넘어간다.

즉 한 동작이 끝나기 전에 다음 동작이나 RL 속도 명령이 끼어들지 않게 직렬화하는
규칙이며, pending 동안 velocity는 항상 0/0/0이다.
*/
