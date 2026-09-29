#include <cassert>

#include "vision_core/c_api.h"
#include "vision_core/control_command.hpp"
#include "vision_core/config_loader.hpp"

int main() {
  using namespace vision_core;

  ControlCommandConfig velocity_config =
      LoadDefaultAlgorithmConfig().command;
  velocity_config.locomotion_backend = LocomotionBackend::kVelocity;
  ControlCommandCoordinator coordinator(velocity_config);
  BallResult ball;
  GoalResult goal;
  HurdleResult hurdle;
  const MotionCommand line{0.5, 0.0, 0.1};

  auto command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);
  assert(command.mission == MissionType::kLine);
  assert(command.velocity.vx == 0.5);
  assert(command.pre_p2p_motion.vx == 0.5);

  hurdle.active = true;
  hurdle.mode = HurdleMode::kRlStopping;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);
  assert(command.control_phase == ControlPhase::kRlStopping);
  assert(command.velocity.vx == 0.0);

  hurdle.mode = HurdleMode::kContactWalk;
  hurdle.action_request = HurdleActionRequest::kContactWalk;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kStepForwardOne);
  assert(command.action_execution_kind ==
         ActionExecutionKind::kVelocityCompatible);
  const auto first_id = command.action_id;
  assert(first_id != 0);

  command = coordinator.Compute(ball, hurdle, goal, line,
                                {first_id, true, false});
  assert(command.command_type == CommandType::kHold);
  assert(command.control_phase == ControlPhase::kWaitingActionDone);
  assert(command.action_id == first_id);

  command = coordinator.Compute(ball, hurdle, goal, line,
                                {first_id, true, true});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == 0);

  // 접촉 전진 DONE 직후 Cross 단계는 다른 action 코드이므로 억제되지 않고
  // 실제 허들 넘기 동작(16)을 새 ID로 발행한다.
  hurdle.mode = HurdleMode::kCross;
  hurdle.action_request = HurdleActionRequest::kCross;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kHurdle);
  assert(command.action_execution_kind == ActionExecutionKind::kDiscrete);
  const auto cross_id = command.action_id;
  assert(cross_id != first_id);
  command = coordinator.Compute(ball, hurdle, goal, line,
                                {cross_id, true, true});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == 0);

  hurdle.action_request = HurdleActionRequest::kNone;
  hurdle.active = false;
  hurdle.mode = HurdleMode::kLineFollow;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);

  hurdle.active = true;
  hurdle.mode = HurdleMode::kContactWalk;
  hurdle.action_request = HurdleActionRequest::kContactWalk;
  hurdle.command = {0.10, 0.0, 0.0};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.action_id != first_id);

  coordinator.Reset();
  hurdle = {};
  ball = {};
  ball.active = true;
  ball.mode = BallMode::kFineAdjustForPickup;
  ball.action_request = BallActionRequest::kFineAdjustForward;
  ball.command = {0.15, 0.0, 0.0};
  command = coordinator.Compute(ball, hurdle, {}, {});
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kStepForwardHalf);
  assert(command.action_execution_kind ==
         ActionExecutionKind::kVelocityCompatible);
  assert(command.pre_p2p_motion.vx == 0.10);
  assert(command.action_yaw_rad == 0.0);
  assert(command.pre_p2p_motion.vx == 0.15);

  coordinator.Reset();
  hurdle = {};
  ball.active = true;
  ball.mode = BallMode::kVerifyPickup;
  ball.action_request = BallActionRequest::kVerifyPickup;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.mission == MissionType::kBall);
  assert(command.mission_phase == static_cast<int>(BallMode::kVerifyPickup));
  assert(command.action == MissionAction::kRecatch);
  assert(command.velocity.vx == 0.0);
  assert(command.pre_p2p_motion.vx == 0.0);
  assert(command.action_execution_kind == ActionExecutionKind::kDiscrete);

  coordinator.Reset();
  goal = {};
  goal.active = true;
  goal.mode = GoalMode::kShoot;
  goal.action_request = GoalActionRequest::kShoot;
  goal.shoot_yaw_rad = -0.35;
  command = coordinator.Compute({}, {}, goal, {});
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kShoot);
  assert(command.action_execution_kind == ActionExecutionKind::kDiscrete);
  assert(command.pre_p2p_motion.vx == 0.0);
  assert(command.action_yaw_rad == -0.35);

  const auto shoot_id = command.action_id;
  goal.shoot_yaw_rad = 0.20;
  command = coordinator.Compute({}, {}, goal, {});
  assert(command.action_id == shoot_id);
  assert(command.action_yaw_rad == -0.35);

  coordinator.Reset();
  ball = {};
  goal.active = true;
  goal.mode = GoalMode::kFineAdjust;
  goal.action_request = GoalActionRequest::kFineAdjustHold;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.mission == MissionType::kGoal);
  assert(command.action == MissionAction::kDefaultPoseMode);
  assert(command.velocity.vx == 0.0);
  assert(command.pre_p2p_motion.vx == 0.0);
  assert(command.action_execution_kind == ActionExecutionKind::kStationary);
  const auto first_hold_id = command.action_id;

  // 반복 측정용 HOLD는 DONE 뒤 같은 요청이 계속되어도 새 ID로 다시
  // 발급되어야 한다.
  command = coordinator.Compute(ball, hurdle, goal, line,
                                {first_hold_id, true, true});
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kDefaultPoseMode);
  assert(command.action_id != first_hold_id);

  coordinator.Reset();
  goal = {};
  ball = {};
  hurdle.active = true;
  hurdle.mode = HurdleMode::kApproach;
  ball.mode = BallMode::kPostPickupLineRecovery;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);
  assert(command.mission == MissionType::kBall);
  assert(command.mission_phase ==
         static_cast<int>(BallMode::kPostPickupLineRecovery));
  assert(command.velocity.vx == 0.0);
  assert(command.velocity.wz == line.wz);

  coordinator.Reset();
  ball = {};
  hurdle = {};
  ball.active = true;
  ball.mode = BallMode::kFineAdjustForPickup;
  ball.action_request = BallActionRequest::kFineAdjustForward;
  hurdle.active = true;
  hurdle.mode = HurdleMode::kContactWalk;
  hurdle.action_request = HurdleActionRequest::kContactWalk;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kHurdle);
  assert(command.action == MissionAction::kStepForwardOne);

  // Ball이 먼저 선택되면 Goal/Hurdle이 중간에 활성화돼도 Ball의 명시적인
  // LINE_FOLLOW 이탈 전에는 최종 미션을 바꾸지 않는다.
  coordinator.Reset();
  ball = {};
  hurdle = {};
  goal = {};
  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.2, 0.0, 0.1};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kBall);

  hurdle.active = true;
  hurdle.mode = HurdleMode::kApproach;
  hurdle.command = {0.3, 0.0, -0.1};
  goal.active = true;
  goal.mode = GoalMode::kApproach;
  goal.command = {0.1, 0.0, 0.0};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kBall);
  assert(command.velocity.vx == 0.2);

  // Ball이 정상 종료된 프레임부터 다음 대기 미션을 선택할 수 있다.
  ball = {};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kGoal);

  // Goal과 Hurdle도 같은 방식으로 자신의 종료 전까지 잠긴다.
  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kGoal);

  goal = {};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kHurdle);
  goal.active = true;
  goal.mode = GoalMode::kApproach;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kHurdle);

  coordinator.Reset();
  goal = {};
  hurdle = {};
  ball.has_ball = true;
  ball.mode = BallMode::kStandUpAfterPickup;
  ball.action_request = BallActionRequest::kStandUp;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kBall);
  assert(command.action == MissionAction::kDefaultPosition);

  // P2P backend는 기존 controller의 연속속도를 보행 ACTION으로 바꾸고,
  // 동일 primitive도 DONE 뒤 새 ID로 다시 실행할 수 있다.
  ControlCommandConfig p2p_config =
      LoadDefaultAlgorithmConfig().command;
  p2p_config.first_action_id = 100;
  p2p_config.locomotion_backend = LocomotionBackend::kP2pAction;
  ControlCommandCoordinator p2p_coordinator(p2p_config);
  ball = {};
  hurdle = {};
  goal = {};
  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action == MissionAction::kStepForwardFive);
  assert(command.action_id == 100);
  assert(command.velocity.vx == 0.0);
  assert(command.pre_p2p_motion.vx == 0.30);
  assert(command.action_execution_kind ==
         ActionExecutionKind::kVelocityCompatible);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_id == 100);
  assert(command.pre_p2p_motion.vx == 0.30);

  // 다른 action_id의 ACK/DONE은 현재 상태를 바꾸지 않는다.
  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.10, 0.0, 0.0},
                                    {999, true, true});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_id == 100);
  assert(command.pre_p2p_motion.vx == 0.30);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {100, true, false});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action_id == 100);
  assert(command.pre_p2p_motion.vx == 0.30);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {100, true, true});
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kStepForwardFive);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action_id == 101);
  assert(command.pre_p2p_motion.vx == 0.30);

  // READY를 받은 긴 라인 보행은 현재 동작이 끝나기 전에 다음 action을
  // 1단계 큐 명령으로 발행한다.
  ControlCommandCoordinator ready_coordinator(p2p_config);
  command = ready_coordinator.Compute({}, {}, {},
                                      MotionCommand{0.30, 0.0, 0.0});
  const auto ready_current_id = command.action_id;
  command = ready_coordinator.Compute({}, {}, {},
                                      MotionCommand{0.30, 0.0, 0.0},
                                      {ready_current_id, true, false, false});
  assert(command.command_type == CommandType::kHold);
  command = ready_coordinator.Compute({}, {}, {},
                                      MotionCommand{0.30, 0.0, -0.35},
                                      {ready_current_id, true, false, true});
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kTurnRightAndStep);
  assert(command.action_id != ready_current_id);
  assert(command.action_yaw_rad == 0.35);
  const auto queued_id = command.action_id;

  command = ready_coordinator.Compute({}, {}, {},
                                      MotionCommand{0.30, 0.0, -0.35},
                                      {queued_id, true, false, false});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == queued_id);
  assert(command.action_yaw_rad == 0.35);

  command = ready_coordinator.Compute({}, {}, {},
                                      MotionCommand{0.30, 0.0, -0.35},
                                      {ready_current_id, true, true, false});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == queued_id);
  assert(command.action_yaw_rad == 0.35);

  // 단발 시험 게이트가 닫혀 있으면 READY를 받아도 다음 action을 예약하지
  // 않고 현재 action의 DONE만 기다린다.
  ControlCommandCoordinator gated_ready_coordinator(p2p_config);
  command = gated_ready_coordinator.Compute(
      {}, {}, {}, MotionCommand{0.30, 0.0, 0.0}, {}, 0.0, false);
  const auto gated_id = command.action_id;
  command = gated_ready_coordinator.Compute(
      {}, {}, {}, MotionCommand{0.30, 0.0, -0.35},
      {gated_id, true, false, true}, 1.0, false);
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == gated_id);
  assert(command.control_phase == ControlPhase::kWaitingActionDone);

  // 영상 왼쪽으로 향해야 하는 조향 의도는 로봇 yaw 좌회전(+)으로 전달한다.
  ControlCommandCoordinator left_yaw_coordinator(p2p_config);
  command = left_yaw_coordinator.Compute({}, {}, {},
                                         MotionCommand{0.30, 0.0, 0.35});
  assert(command.action == MissionAction::kTurnLeftAndStep);
  assert(command.action_yaw_rad == 0.35);

  // 중간 조향은 고정 4걸음 곡선 모션이며 별도 목표 yaw를 전달하지 않는다.
  ControlCommandCoordinator curve_four_coordinator(p2p_config);
  command = curve_four_coordinator.Compute({}, {}, {},
                                           MotionCommand{0.30, 0.0, -0.20});
  assert(command.action == MissionAction::kStepForwardRight);
  assert(command.action_yaw_rad == 0.0);

  // 저속에서 큰 조향은 제자리회전이며 반복 횟수 결정을 위한 yaw를 전달한다.
  ControlCommandCoordinator in_place_coordinator(p2p_config);
  command = in_place_coordinator.Compute({}, {}, {},
                                         MotionCommand{0.10, 0.0, 0.20});
  assert(command.action == MissionAction::kTurnLeft);
  assert(command.action_yaw_rad == 0.20);

  // 라인 속도 의도가 모두 deadband 안이면 별도 모션을 발행하지
  // 않고 현재 자세를 유지한다.
  ControlCommandCoordinator hold_pose_coordinator(p2p_config);
  command = hold_pose_coordinator.Compute({}, {}, {}, MotionCommand{});
  assert(command.command_type == CommandType::kHold);
  assert(command.action == MissionAction::kNone);
  assert(command.action_id == 0);
  assert(command.action_category == ActionCategory::kNone);

  // ACK가 설정 시간 안에 오지 않으면 늦은 ACK에 의한 중복 실행을 피하도록
  // 자동 재발행하지 않고 Reset 전까지 timeout HOLD로 잠근다.
  ControlCommandCoordinator timeout_coordinator(p2p_config);
  command = timeout_coordinator.Compute({}, {}, {},
                                        MotionCommand{0.30, 0.0, 0.0}, {},
                                        1.0);
  const auto timeout_id = command.action_id;
  command = timeout_coordinator.Compute({}, {}, {},
                                        MotionCommand{0.30, 0.0, 0.0}, {},
                                        10.999);
  assert(command.command_type == CommandType::kAction);
  assert(command.action_id == timeout_id);
  command = timeout_coordinator.Compute({}, {}, {},
                                        MotionCommand{0.30, 0.0, 0.0}, {},
                                        11.0);
  assert(command.command_type == CommandType::kHold);
  assert(command.control_phase == ControlPhase::kActionAckTimedOut);
  assert(command.action_id == timeout_id);
  command = timeout_coordinator.Compute(
      {}, {}, {}, MotionCommand{0.30, 0.0, 0.0},
      {timeout_id, true, false, false}, 11.1);
  assert(command.command_type == CommandType::kHold);
  assert(command.control_phase == ControlPhase::kActionAckTimedOut);
  timeout_coordinator.Reset();
  command = timeout_coordinator.Compute({}, {}, {},
                                        MotionCommand{0.30, 0.0, 0.0}, {},
                                        11.2);
  assert(command.command_type == CommandType::kAction);
  assert(command.action_id != timeout_id);

  // 진행 중 보행은 ID가 완료될 때까지 직렬화한다. 완료된 프레임에는 새
  // 보행보다 controller가 요청한 미션 액션을 우선 발급한다.
  ball.active = true;
  ball.mode = BallMode::kPickupBall;
  ball.action_request = BallActionRequest::kPickup;
  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0});
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action_id == 101);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {101, true, true});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_category == ActionCategory::kMission);
  assert(command.action == MissionAction::kPickBall);
  const auto pickup_id = command.action_id;
  assert(pickup_id == 102);
  assert(command.action_execution_kind == ActionExecutionKind::kDiscrete);
  assert(command.pre_p2p_motion.vx == 0.0);

  // 미션 액션은 DONE 뒤 controller가 요청을 내릴 때까지 같은 상태에서
  // 재발급하지 않는다.
  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {pickup_id, true, true});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == 0);
  assert(command.action_category == ActionCategory::kNone);

  ball.action_request = BallActionRequest::kNone;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.30, 0.0, -0.20};
  command = p2p_coordinator.Compute(ball, hurdle, goal, {});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action == MissionAction::kTurnRightAndStep);
  assert(command.action_yaw_rad == 0.20);

  // P2P의 영속 정지는 /cmd_vel 대신 새 액션을 만들지 않는 HOLD다.
  ControlCommandCoordinator p2p_idle(p2p_config);
  command = p2p_idle.Compute({}, {}, {}, {});
  assert(command.command_type == CommandType::kHold);
  assert(command.action == MissionAction::kNone);
  assert(command.action_id == 0);
  assert(command.velocity.vx == 0.0);

  // Coordinator가 선택한 mission+phase가 P2P 프로필까지 전달된다.
  ControlCommandCoordinator p2p_profile_coordinator(p2p_config);
  ball = {};
  ball.active = true;
  ball.mode = BallMode::kTiltCameraDownAndApproach;
  ball.command = {0.30, 0.0, 0.0};
  command = p2p_profile_coordinator.Compute(ball, {}, {}, {});
  assert(command.command_type == CommandType::kAction);
  assert(command.mission == MissionType::kBall);
  assert(command.mission_phase ==
         static_cast<int>(BallMode::kTiltCameraDownAndApproach));
  assert(command.action == MissionAction::kStepForwardOne);

  // Goal 미세보행은 Mission action이지만 실행기 관점에서는 PRE-P2P
  // velocity로 대체할 수 있고, ACK/DONE은 같은 action_id로 유지된다.
  ControlCommandCoordinator fine_coordinator(p2p_config);
  goal = {};
  goal.active = true;
  goal.mode = GoalMode::kFineAdjust;
  goal.action_request = GoalActionRequest::kFineAdjust;
  goal.command = {0.0, 0.12, 0.0};
  command = fine_coordinator.Compute({}, {}, goal, {});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_category == ActionCategory::kMission);
  assert(command.action == MissionAction::kLeftSideStep);
  assert(command.action_execution_kind ==
         ActionExecutionKind::kVelocityCompatible);
  assert(command.pre_p2p_motion.vy == 0.12);
  const auto fine_id = command.action_id;
  command = fine_coordinator.Compute({}, {}, goal, {},
                                     {fine_id, true, false});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == fine_id);
  assert(command.pre_p2p_motion.vy == 0.12);

  // 확장 C API도 동일한 최종 action, PRE-P2P 속도, 실행 정책을 한 번의
  // 상태 전이 결과로 반환한다. 기존 v1/v2 구조체는 변경하지 않는다.
  auto c_handle = vision_control_command_coordinator_create();
  const VisionControlCommandV3 c_command = vision_control_command_compute_v3(
      c_handle, VisionBallResult{}, VisionHurdleResult{}, VisionGoalResult{},
      0, 0.30, 0.0, 0.0, 0, 0, 0);
  assert(c_command.command_type == static_cast<int>(CommandType::kAction));
  assert(c_command.vx == 0.0);
  assert(c_command.pre_p2p_vx == 0.30);
  assert(c_command.action_id != 0);
  assert(c_command.action_category ==
         static_cast<int>(ActionCategory::kLocomotion));
  assert(c_command.action_execution_kind ==
         static_cast<int>(ActionExecutionKind::kVelocityCompatible));
  vision_control_command_coordinator_destroy(c_handle);
  return 0;
}
