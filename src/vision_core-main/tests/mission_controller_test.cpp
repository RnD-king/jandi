#include "vision_core/mission_controller.hpp"
#include "vision_core/config_loader.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

namespace {

using namespace vision_core;

ObjectTarget Target(int class_id, double u_norm, double center_v_norm) {
  ObjectTarget target;
  target.class_id = class_id;
  target.confidence = 1.0;
  target.width_px = 20.0;
  target.height_px = 20.0;
  target.area_px = 400.0;
  target.box_px = {u_norm * 100.0 - 10.0,
                   center_v_norm * 100.0 - 10.0, 20.0, 20.0};
  target.center_px = {u_norm * 100.0, center_v_norm * 100.0};
  target.rectified_center_px = target.center_px;
  return target;
}

MissionFrameInput Frame(double now_sec) {
  MissionFrameInput input;
  input.image_width = 100;
  input.image_height = 100;
  input.now_sec = now_sec;
  input.camera_feedback = {CameraMode::kForward, true};
  input.line_centers = {{50.0, 90.0}, {50.0, 70.0}, {50.0, 50.0},
                        {50.0, 30.0}};
  return input;
}

MissionControllerConfig FastConfig() {
  MissionControllerConfig config = LoadDefaultAlgorithmConfig();
  config.line_features.image_center_u = 50.0;
  config.ball.stable_window = 1;
  config.ball.stable_min_hits = 1;
  config.ball.upper_acquire_v_norm = 0.65;
  config.ball.tilt_down_v_norm = 0.95;
  config.hurdle.stable_window = 1;
  config.hurdle.stable_min_hits = 1;
  config.hurdle.acquire_min_v_norm = 0.60;
  config.hurdle.tilt_trigger_v_norm = 0.95;
  config.goal.stable_window = 1;
  config.goal.stable_min_hits = 1;
  config.goal.post_pickup_wait_sec = 1.0;
  return config;
}

void TestOnlyActiveBallControllerAdvances() {
  MissionController controller(FastConfig());
  auto input = Frame(0.0);
  input.ball_target = Target(1, 0.5, 0.5);
  auto result = controller.Step(input);
  assert(result.active_mission == MissionType::kBall);
  assert(result.ball.mode == BallMode::kApproachBall);

  input = Frame(0.1);
  input.ball_target = Target(1, 0.5, 0.5);
  input.hurdle_target = Target(4, 0.5, 0.7);
  input.goal_target = Target(2, 0.5, 0.4);
  result = controller.Step(input);
  assert(result.active_mission == MissionType::kBall);
  assert(result.goal.mode == GoalMode::kLineFollow);
  assert(result.hurdle.mode == HurdleMode::kLineFollow);
  assert(!result.line_computed);
}

void TestHurdleEntryHasPriorityOverBall() {
  MissionController controller(FastConfig());
  auto input = Frame(0.0);
  input.ball_target = Target(1, 0.5, 0.5);
  input.hurdle_target = Target(4, 0.5, 0.7);
  const auto result = controller.Step(input);
  assert(result.active_mission == MissionType::kHurdle);
  assert(result.hurdle.mode == HurdleMode::kApproach);
  assert(result.ball.mode == BallMode::kLineFollow);
}

void TestCarryingBallAllowsOnlyGoalEntry() {
  auto config = FastConfig();
  config.initial_has_ball = true;
  config.line.line_stable_window = 1;
  config.line.line_stable_min_hits = 1;
  MissionController controller(config);
  auto input = Frame(0.0);
  input.ball_target = Target(1, 0.5, 0.5);
  input.hurdle_target = Target(4, 0.5, 0.7);
  // 공을 들고 라인을 다시 잡으면 ball/hurdle 후보를 무시하고, 백보드가
  // 아직 없어도 설정 시간 동안 라인 미션을 계속한다.
  auto result = controller.Step(input);
  assert(result.active_mission == MissionType::kLine);
  assert(result.goal.mode == GoalMode::kLineFollow);
  assert(result.line_computed);

  input = Frame(0.99);
  result = controller.Step(input);
  assert(result.active_mission == MissionType::kLine);
  assert(result.goal.mode == GoalMode::kLineFollow);

  // 연속 라인 추종 시간이 끝나면 백보드 선행 검출 없이 카메라를 든다.
  input = Frame(1.0);
  result = controller.Step(input);
  assert(result.active_mission == MissionType::kGoal);
  assert(result.goal.mode == GoalMode::kTiltCameraToGoal);
  assert(result.goal.camera_request == CameraRequest::kGoal);
  assert(result.hurdle.mode == HurdleMode::kLineFollow);
  assert(result.ball.mode == BallMode::kLineFollow);
}

void TestMissionControllerReturnsActionAndPreP2pMotionTogether() {
  auto config = FastConfig();
  config.enable_ball = false;
  config.enable_hurdle = false;
  config.enable_goal = false;
  config.line.line_stable_window = 1;
  config.line.line_stable_min_hits = 1;
  config.command.locomotion_backend = LocomotionBackend::kP2pAction;
  config.command.first_action_id = 500;
  MissionController controller(config);

  auto input = Frame(0.0);
  auto result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action_category == ActionCategory::kLocomotion);
  assert(result.command.action_execution_kind ==
         ActionExecutionKind::kVelocityCompatible);
  assert(result.command.action_id == 500);
  assert(result.command.velocity.vx == 0.0);
  assert(result.command.pre_p2p_motion.vx == result.line_command.vx);
  assert(result.command.pre_p2p_motion.vx > 0.0);
  const MotionCommand first_pre_p2p = result.command.pre_p2p_motion;

  // ACK 뒤에도 같은 action_id와 최초 선택 시점의 PRE-P2P 명령을 유지한다.
  input = Frame(0.1);
  input.delivery_feedback = {500, true, false};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.control_phase == ControlPhase::kWaitingActionDone);
  assert(result.command.action_id == 500);
  assert(result.command.pre_p2p_motion.vx == first_pre_p2p.vx);

  // 긴 action 후반에는 오른쪽으로 치우친 line을 관측한다. READY 프레임
  // 자체는 다시 중앙선이지만, 예약 판단은 후반 누적 특징을 사용해야 한다.
  input = Frame(0.2);
  for (auto &point : input.line_centers) point.u = 70.0;
  input.delivery_feedback = {500, false, false};
  result = controller.Step(input);
  assert(result.command.action_id == 500);

  input = Frame(0.3);
  for (auto &point : input.line_centers) point.u = 70.0;
  input.delivery_feedback = {500, false, false};
  result = controller.Step(input);
  assert(result.command.action_id == 500);

  input = Frame(0.4);
  input.delivery_feedback = {500, true, false, true};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action_id == 501);
  assert(result.command.action == MissionAction::kStepForwardRight);

  // 현재 action의 실제 DONE 뒤에는 예약 action이 실행 중 action으로 승격된다.
  input = Frame(0.5);
  input.delivery_feedback = {500, true, true, false};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action_id == 501);
}

void TestShortLineActionCollectsForOneSecondAfterDone() {
  auto config = FastConfig();
  config.enable_ball = false;
  config.enable_hurdle = false;
  config.enable_goal = false;
  config.line.line_stable_window = 1;
  config.line.line_stable_min_hits = 1;
  config.command.locomotion_backend = LocomotionBackend::kP2pAction;
  // 첫 직진을 1걸음 action으로 만들고 DONE 뒤 1초 관측을 검증한다.
  config.command.p2p.long_forward_vx = 1.0;
  config.line_p2p.short_post_collect_sec = 1.0;
  config.command.first_action_id = 600;
  MissionController controller(config);

  auto input = Frame(0.0);
  auto result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action == MissionAction::kStepForwardOne);
  assert(result.command.action_id == 600);

  input = Frame(0.1);
  input.delivery_feedback = {600, true, false};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.action_id == 600);

  // DONE 직후에는 다음 action을 발행하지 않고 정지 관측을 시작한다.
  input = Frame(2.0);
  for (auto &point : input.line_centers) point.u = 60.0;
  input.delivery_feedback = {600, true, true};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.action_id == 0);

  input = Frame(2.5);
  for (auto &point : input.line_centers) point.u = 60.0;
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.action_id == 0);

  // DONE+1초에 수집한 오른쪽 치우침이 다음 PRE-P2P 조향 의도에 반영된다.
  // 저속에서 조향 임계값을 넘으면 1걸음 직진 대신 제자리회전을 선택한다.
  input = Frame(3.0);
  for (auto &point : input.line_centers) point.u = 60.0;
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action_id == 601);
  assert(result.command.action == MissionAction::kTurnRight);
  assert(result.command.pre_p2p_motion.wz < -0.10);
}

void TestOneShotLineDecisionGateAndSafeTuningUpdate() {
  auto config = FastConfig();
  config.enable_ball = false;
  config.enable_hurdle = false;
  config.enable_goal = false;
  config.line.line_stable_window = 1;
  config.line.line_stable_min_hits = 1;
  config.command.locomotion_backend = LocomotionBackend::kP2pAction;
  config.command.first_action_id = 700;
  MissionController controller(config);

  auto input = Frame(0.0);
  input.allow_new_line_locomotion_action = false;
  auto result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.action_id == 0);

  auto line_p2p = config.line_p2p;
  line_p2p.offset_gain = 2.0;
  assert(controller.UpdateLineP2pTuning(line_p2p, config.command.p2p));

  input = Frame(1.0);
  input.allow_new_line_locomotion_action = true;
  input.line_decision_guide_override = LineGuide{0.20, 0.0, 0.0, 1.0, true};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action == MissionAction::kTurnRightAndStep);
  assert(result.command.action_id == 700);
  // 실행 중인 action이 있으면 gain/기준 교체를 거절한다.
  assert(!controller.UpdateLineP2pTuning(config.line_p2p,
                                         config.command.p2p));
}

void TestMissingLineHoldsWithoutPublishingActionForTwoSeconds() {
  auto config = FastConfig();
  config.enable_ball = false;
  config.enable_hurdle = false;
  config.enable_goal = false;
  config.line.line_stable_window = 1;
  config.line.line_stable_min_hits = 1;
  config.command.locomotion_backend = LocomotionBackend::kP2pAction;
  config.command.first_action_id = 800;
  config.line_p2p.no_action_hold_sec = 2.0;
  MissionController controller(config);

  auto input = Frame(0.0);
  input.line_centers.clear();
  auto result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.action == MissionAction::kNone);
  assert(result.command.action_id == 0);

  // 유지 시간 중 라인이 다시 보여도 중간에 새 action을 발행하지 않는다.
  input = Frame(1.0);
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.action_id == 0);

  // 정확히 2초가 지나면 최신 유효 관측으로 정상 locomotion을 재개한다.
  input = Frame(2.0);
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action == MissionAction::kStepForwardFive);
  assert(result.command.action_id == 800);
}

} // namespace

int main() {
  TestOnlyActiveBallControllerAdvances();
  TestHurdleEntryHasPriorityOverBall();
  TestCarryingBallAllowsOnlyGoalEntry();
  TestMissionControllerReturnsActionAndPreP2pMotionTogether();
  TestShortLineActionCollectsForOneSecondAfterDone();
  TestOneShotLineDecisionGateAndSafeTuningUpdate();
  TestMissingLineHoldsWithoutPublishingActionForTwoSeconds();
  std::cout << "mission controller tests passed\n";
  return 0;
}
