#include "vision_core/line_velocity_controller.hpp"
#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_p2p_controller.hpp"
#include "vision_core/config_loader.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

namespace {

vision_core::Features ReliableLine() {
  vision_core::Features features;
  features.n_visible = 4.0;
  features.u_err_near = 0.0;
  features.u_err_ctrl = 0.0;
  return features;
}

vision_core::Features MissingLine() {
  vision_core::Features features;
  features.n_visible = 0.0;
  features.in_recovery = 1.0;
  return features;
}

void TestLineUsesTenFrameSevenHitRule() {
  vision_core::LineVelocityController controller;
  const auto reliable = ReliableLine();
  for (int frame = 0; frame < 6; ++frame) {
    controller.Observe(reliable, {});
    assert(controller.InRecovery());
  }
  controller.Observe(reliable, {});
  assert(!controller.InRecovery());

  controller.BeginReacquisition();
  assert(controller.InRecovery());
  for (int frame = 0; frame < 6; ++frame) {
    controller.Observe(reliable, {});
    assert(controller.InRecovery());
  }
  controller.Observe(reliable, {});
  assert(!controller.InRecovery());

  const auto missing = MissingLine();
  for (int frame = 0; frame < 3; ++frame) {
    controller.Observe(missing, {});
    assert(!controller.InRecovery());
  }
  controller.Observe(missing, {});
  assert(controller.InRecovery());
}

void TestVisibleRecoveryUsesCurrentLookahead() {
  vision_core::LineVelocityController controller;
  auto features = ReliableLine();
  features.u_err_ctrl = 0.5;

  controller.BeginReacquisition();
  const auto tracking_command = controller.Compute(features);

  assert(controller.InRecovery());
  assert(tracking_command.wz < 0.0);
}

void TestLowVisibilityRecoveryUsesPathMemory() {
  vision_core::LineVelocityController controller;
  auto features = ReliableLine();
  features.n_visible = 1.0;
  features.u_err_ctrl = 0.5;

  controller.BeginReacquisition();
  const auto command = controller.Compute(features);

  assert(controller.InRecovery());
  assert(command.wz > 0.0);
}

void TestSearchRotationDoesNotWalkForward() {
  vision_core::LineVelocityController controller;
  controller.BeginReacquisition();
  const auto command = controller.ComputeSearchRotation();
  assert(command.vx == 0.0);
  assert(command.wz != 0.0);
}

void TestLookaheadApproachUsesOnlyFarPoint() {
  vision_core::LineVelocityController controller;
  auto features = ReliableLine();
  features.u_err_near = -0.9;
  features.u_err_lookahead = 0.4;
  features.u_err_ctrl = -0.7;
  features.slope = -1.0;
  const auto command = controller.ComputeLookaheadApproach(features);
  assert(command.vx > 0.0 && command.vx <= 0.45);
  assert(command.wz < 0.0);
}

void TestStraightKeepsCurveScoreNearZero() {
  const auto config =
      vision_core::LoadDefaultAlgorithmConfig().line_features;
  const std::vector<vision_core::Point2> points{
      {320.0, 420.0}, {320.0, 350.0}, {320.0, 280.0},
      {320.0, 210.0}, {320.0, 140.0}, {320.0, 70.0},
  };
  vision_core::LineFeatureState state;
  for (int frame = 0; frame < 10; ++frame) {
    const auto features = vision_core::ComputeLineFeatures(
        points, 640, 480, false, 0.0, 0.0, config, &state);
    assert(std::abs(features.u_err_lookahead) < 1e-9);
    assert(features.guide.valid);
    assert(std::abs(features.guide.offset) < 1e-9);
    assert(std::abs(features.guide.heading_rad) < 1e-9);
    assert(std::abs(features.guide.curvature_rad) < 1e-9);
    assert(features.guide.confidence > 0.99);
  }
  assert(state.filtered_curve_score < 1e-9);
}

void TestCurveUsesStableAdaptiveLookaheadAndSparseFallback() {
  const auto config =
      vision_core::LoadDefaultAlgorithmConfig().line_features;
  const std::vector<vision_core::Point2> curve_points{
      {320.0, 420.0}, {322.0, 350.0}, {330.0, 280.0},
      {350.0, 210.0}, {385.0, 140.0}, {435.0, 70.0},
  };
  vision_core::LineFeatureState state;
  vision_core::Features curve_features;
  for (int frame = 0; frame < 10; ++frame) {
    curve_features = vision_core::ComputeLineFeatures(
        curve_points, 640, 480, false, 0.0, 0.0, config, &state);
  }
  assert(state.filtered_curve_score > 0.1);
  assert(curve_features.u_err_lookahead > 0.0);
  assert(curve_features.slope > 0.0);
  assert(curve_features.guide.valid);
  assert(curve_features.guide.heading_rad > 0.0);
  assert(curve_features.guide.curvature_rad > 0.05);
  assert(curve_features.guide.confidence > 0.0);

  const auto algorithm = vision_core::LoadDefaultAlgorithmConfig();
  const vision_core::LineP2pController p2p(algorithm.line_p2p,
                                            algorithm.line);
  assert(p2p.Compute(curve_features.guide).wz < 0.0);

  std::vector<vision_core::Point2> left_curve = curve_points;
  for (auto &point : left_curve) point.u = 640.0 - point.u;
  const auto left_features = vision_core::ComputeLineFeatures(
      left_curve, 640, 480, false, 0.0, 0.0, config);
  assert(left_features.u_err_lookahead < 0.0);
  assert(left_features.slope < 0.0);
  assert(left_features.guide.heading_rad < 0.0);
  assert(left_features.guide.curvature_rad < -0.05);
  assert(p2p.Compute(left_features.guide).wz > 0.0);

  const double before_sparse = state.filtered_curve_score;
  const std::vector<vision_core::Point2> sparse_points{
      {320.0, 420.0}, {250.0, 260.0}, {180.0, 100.0},
  };
  vision_core::ComputeLineFeatures(
      sparse_points, 640, 480, false, 0.0, 0.0, config, &state);
  assert(state.filtered_curve_score < before_sparse);
  assert(state.filtered_curve_score > before_sparse * 0.90);
}

void TestGuideOffsetAndLocalFitConfidence() {
  const auto config =
      vision_core::LoadDefaultAlgorithmConfig().line_features;
  const std::vector<vision_core::Point2> shifted_straight{
      {400.0, 420.0}, {400.0, 350.0}, {400.0, 280.0},
      {400.0, 210.0}, {400.0, 140.0}, {400.0, 70.0},
  };
  const auto straight = vision_core::ComputeLineFeatures(
      shifted_straight, 640, 480, false, 0.0, 0.0, config);
  assert(straight.guide.valid);
  assert(std::abs(straight.guide.offset - 0.25) < 1e-9);
  assert(std::abs(straight.guide.heading_rad) < 1e-9);

  const std::vector<vision_core::Point2> scattered{
      {280.0, 420.0}, {360.0, 350.0}, {285.0, 280.0},
      {355.0, 210.0}, {290.0, 140.0}, {350.0, 70.0},
  };
  const auto noisy = vision_core::ComputeLineFeatures(
      scattered, 640, 480, false, 0.0, 0.0, config);
  assert(noisy.guide.valid);
  assert(noisy.guide.confidence < straight.guide.confidence);
}

} // namespace

int main() {
  TestLineUsesTenFrameSevenHitRule();
  TestVisibleRecoveryUsesCurrentLookahead();
  TestLowVisibilityRecoveryUsesPathMemory();
  TestSearchRotationDoesNotWalkForward();
  TestLookaheadApproachUsesOnlyFarPoint();
  TestStraightKeepsCurveScoreNearZero();
  TestCurveUsesStableAdaptiveLookaheadAndSparseFallback();
  TestGuideOffsetAndLocalFitConfidence();
  std::cout << "line detection stability tests passed\n";
  return 0;
}
