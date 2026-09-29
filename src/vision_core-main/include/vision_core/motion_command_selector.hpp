#pragma once

#include "vision_core/ball_controller.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_velocity_controller.hpp"

namespace vision_core {

enum class CommandSource {
  kLine = 0,
  kBall,
  kHurdle,
  kGoal,
};

struct SelectedMotionCommand {
  MotionCommand command;
  CommandSource source{CommandSource::kLine};
  BallMode ball_mode{BallMode::kLineFollow};
  HurdleMode hurdle_mode{HurdleMode::kLineFollow};
  GoalMode goal_mode{GoalMode::kLineFollow};
};

// Pure selection overload for callers that already updated and computed the
// line controller exactly once this frame.
SelectedMotionCommand SelectMotionCommand(
    const BallResult &ball_result, const MotionCommand &line_candidate);

// 통합 미션 선택: 명시적으로 시작되는 골대 미션을 가장 먼저 유지하고,
// 공을 들고 있지 않은 일반 라인 주행에서는 허들을 공보다 우선한다.
// 공 집기 성공부터 슛 완료까지는 허들을 선택하지 않는다.
SelectedMotionCommand SelectMotionCommand(
    const BallResult &ball_result, const HurdleResult &hurdle_result,
    const GoalResult &goal_result, const MotionCommand &line_candidate);

// Compatibility overload. It computes the line candidate even while the ball
// command is active. The caller must Observe() once beforehand and must not
// also call Compute() in the same frame; precomputed callers use the pure
// overload above.
SelectedMotionCommand SelectMotionCommand(
    const BallResult &ball_result, const Features &line_features,
    LineVelocityController &line_controller);

const char *SelectedModeName(const SelectedMotionCommand &selected);

} // namespace vision_core
