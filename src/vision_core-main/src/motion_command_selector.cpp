// 처리 순서: [최종 4단계] 점선 분기와 물체(공) 분기가 각각 후보 명령을 만든 뒤 호출한다.
// 역할: 공 제어가 active이면 공 명령을, 아니면 점선 명령을 최종 MotionCommand로 선택한다.
// 값 전달: BallResult와 점선 MotionCommand를 인자로 받고 SelectedMotionCommand를 반환한다.
// 이후 처리: 실제 vision 노드는 이를 ROS2 Twist로, G1은 C API 결과로 사용한다.

#include "vision_core/motion_command_selector.hpp"

namespace vision_core {

SelectedMotionCommand SelectMotionCommand(
    const BallResult &ball_result, const MotionCommand &line_candidate) {
  SelectedMotionCommand selected;
  selected.ball_mode = ball_result.mode;
  if (ball_result.active) {
    selected.command = ball_result.command;
    selected.source = CommandSource::kBall;
  } else {
    selected.command = line_candidate;
    selected.source = CommandSource::kLine;
  }
  return selected;
}

SelectedMotionCommand SelectMotionCommand(
    const BallResult &ball_result, const HurdleResult &hurdle_result,
    const GoalResult &goal_result, const MotionCommand &line_candidate) {
  SelectedMotionCommand selected;
  selected.ball_mode = ball_result.mode;
  selected.hurdle_mode = hurdle_result.mode;
  selected.goal_mode = goal_result.mode;
  // 골대 미션은 공 집기 뒤 명시적으로 시작되므로 HEADING_RECOVERY까지 잠근다.
  // recovery에서는 goal_result.active=false지만 다른 객체로 넘어가지 않고
  // line controller의 복구 후보를 그대로 사용해야 한다.
  if (goal_result.mode != GoalMode::kLineFollow) {
    if (goal_result.active) {
      selected.command = goal_result.command;
      selected.source = CommandSource::kGoal;
    } else {
      selected.command = line_candidate;
      selected.source = CommandSource::kLine;
    }
  } else if (ball_result.has_ball) {
    // 집기 성공부터 슛 완료까지는 허들로 진입하지 않는다. 공 미션의 마무리
    // 상태가 active이면 Ball 명령을, 종료 뒤에는 Line 명령을 사용한다.
    if (ball_result.active) {
      selected.command = ball_result.command;
      selected.source = CommandSource::kBall;
    } else if (ball_result.mode == BallMode::kPostPickupLineRecovery) {
      selected.command = line_candidate;
      selected.command.vx = 0.0;
      selected.command.vy = 0.0;
      selected.source = CommandSource::kLine;
    } else {
      selected.command = line_candidate;
      selected.source = CommandSource::kLine;
    }
  } else if (hurdle_result.active) {
    selected.command = hurdle_result.command;
    selected.source = CommandSource::kHurdle;
  } else if (ball_result.active) {
    selected.command = ball_result.command;
    selected.source = CommandSource::kBall;
  } else if (ball_result.mode == BallMode::kPostPickupLineRecovery) {
    selected.command = line_candidate;
    selected.command.vx = 0.0;
    selected.command.vy = 0.0;
    selected.source = CommandSource::kLine;
  } else {
    selected.command = line_candidate;
    selected.source = CommandSource::kLine;
  }
  return selected;
}

SelectedMotionCommand SelectMotionCommand(
    const BallResult &ball_result, const Features &line_features,
    LineVelocityController &line_controller) {
  const auto line_command = line_controller.Compute(line_features);
  return SelectMotionCommand(
      ball_result, {line_command.vx, 0.0, line_command.wz});
}

const char *SelectedModeName(const SelectedMotionCommand &selected) {
  if (selected.source == CommandSource::kBall) {
    return BallController::ModeName(selected.ball_mode);
  }
  if (selected.source == CommandSource::kHurdle) {
    return HurdleController::ModeName(selected.hurdle_mode);
  }
  if (selected.source == CommandSource::kGoal) {
    return GoalController::ModeName(selected.goal_mode);
  }
  return "LINE_RULE";
}

} // namespace vision_core
