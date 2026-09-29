#include "vision_core/mission_controller.hpp"
#include "vision_core/config_loader.hpp"
#include "vision_core/c_api.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>

namespace {

using namespace vision_core;

PerceptionDetection Observation(int class_id, double confidence,
                                Box2 box) {
  PerceptionDetection value;
  value.detection = {box, confidence, class_id};
  return value;
}

MissionControllerConfig Config() {
  MissionControllerConfig config = LoadDefaultAlgorithmConfig();
  config.line_features.image_center_u = 50.0;
  config.line_detection = {0, 0.60, 1.0, 1.0};
  config.object_targets.ball_confidence = 0.60;
  config.object_targets.goal_confidence = 0.60;
  config.object_targets.backboard_confidence = 0.60;
  config.object_targets.hurdle_confidence = 0.60;
  config.goal.fine_adjust_start_z_m = 2.0;
  config.enable_ball = false;
  config.enable_hurdle = false;
  config.enable_goal = false;
  return config;
}

PerceptionFrameInput Frame() {
  PerceptionFrameInput input;
  input.intrinsics = {100.0, 100.0, 50.0, 50.0};
  input.image_width = 100;
  input.image_height = 100;
  input.camera_feedback = {CameraMode::kForward, true};
  input.detections = {
      Observation(0, 0.90, {40.0, 79.0, 20.0, 20.0}),
      Observation(0, 0.90, {40.0, 39.0, 20.0, 20.0}),
      Observation(1, 0.80, {40.0, 50.0, 20.0, 30.0}),
      Observation(3, 0.90, {30.0, 20.0, 40.0, 20.0}),
  };
  input.detections[3].left_depth_m = 1.0;
  input.detections[3].right_depth_m = 1.2;
  input.detections[3].center_depth_m = 1.1;
  return input;
}

void TestRawExtractionAndGoalDepth() {
  MissionController controller(Config());
  const auto result = controller.StepPerception(Frame());
  assert(result.perception.raw_line_centers.size() == 2);
  assert(result.perception.raw_line_centers[0].v == 89.0);
  assert(result.perception.targets.ball.has_value());
  assert(result.perception.targets.ball->center_px.v == 65.0);
  assert(!result.perception.targets.ball->center_rectified);
  assert(result.perception.goal_pose.valid);
  assert(!result.perception.imu_rectification_applied);
}

void TestPerceptionUsesCameraPrincipalPointForLineCenter() {
  auto config = Config();
  // YAML/C++ fallback과 다른 principal point를 넣어 perception 경로가 실제
  // 카메라 intrinsics를 우선하는지 확인한다.
  config.line_features.image_center_u = 50.0;
  MissionController controller(config);
  auto input = Frame();
  input.intrinsics.cx = 55.0;
  for (auto &detection : input.detections) {
    if (detection.detection.class_id == 0) {
      detection.detection.box.x += 5.0;
    }
  }
  const auto result = controller.StepPerception(input);
  assert(std::abs(result.mission.line_features.u_err_near) < 1e-12);
  assert(std::abs(result.mission.line_features.guide.offset) < 1e-12);
}

void TestBestBackboardKeepsMatchingDepth() {
  MissionController controller(Config());
  auto input = Frame();
  input.detections[3].left_depth_m = 3.0;
  input.detections[3].right_depth_m = 3.0;
  input.detections[3].center_depth_m = 3.0;
  auto selected = Observation(3, 0.95, {20.0, 20.0, 60.0, 20.0});
  selected.left_depth_m = 0.8;
  selected.right_depth_m = 1.0;
  selected.center_depth_m = 0.9;
  input.detections.push_back(selected);
  const auto result = controller.StepPerception(input);
  assert(result.perception.targets.backboard->confidence == 0.95);
  assert(result.perception.backboard_center_depth_m == 0.9);
  assert(result.perception.backboard_left_depth_m == 0.8);
  assert(result.perception.backboard_right_depth_m == 1.0);
  assert(result.perception.goal_pose.valid);
  assert(result.perception.goal_pose.z_m < 1.0);
}

void TestBackboardCenterDepthRejectsHigherConfidenceFalsePositive() {
  MissionController controller(Config());
  auto input = Frame();
  input.detections[3].detection.confidence = 0.99;
  input.detections[3].center_depth_m = 8.0;

  auto valid = Observation(3, 0.80, {20.0, 20.0, 60.0, 20.0});
  valid.center_depth_m = 1.5;
  valid.left_depth_m = 1.4;
  valid.right_depth_m = 1.6;
  input.detections.push_back(valid);

  const auto result = controller.StepPerception(input);
  assert(result.perception.targets.backboard.has_value());
  assert(result.perception.targets.backboard->confidence == 0.80);
  assert(result.perception.backboard_center_depth_m == 1.5);
  assert(result.perception.goal_pose.valid);
  assert(result.perception.goal_pose.z_m == 1.5);
}

void TestImuCanBeSkippedOrApplied() {
  MissionController no_imu_controller(Config());
  auto input = Frame();
  input.enable_imu_rectification = false;
  input.imu_valid = true;
  input.roll_rad = 0.1;
  const auto skipped = no_imu_controller.StepPerception(input);
  assert(!skipped.perception.imu_rectification_applied);
  assert(skipped.perception.line_centers[0].u ==
         skipped.perception.raw_line_centers[0].u);

  MissionController missing_imu_controller(Config());
  input.enable_imu_rectification = true;
  input.imu_valid = false;
  const auto missing = missing_imu_controller.StepPerception(input);
  assert(!missing.perception.imu_rectification_applied);

  MissionController with_imu_controller(Config());
  input.imu_valid = true;
  const auto applied = with_imu_controller.StepPerception(input);
  assert(applied.perception.imu_rectification_applied);
  assert(applied.perception.targets.ball->center_rectified);
  assert(applied.perception.targets.ball->center_px.v == 65.0);
}

void TestEmptyDetectionFrameStillSteps() {
  MissionController controller(Config());
  PerceptionFrameInput input;
  input.image_width = 100;
  input.image_height = 100;
  const auto result = controller.StepPerception(input);
  assert(result.perception.raw_line_centers.empty());
  assert(!result.perception.targets.ball);
  assert(result.mission.active_mission == MissionType::kLine);
}

void TestMissionPerceptionCapiUsesSharedPipeline() {
  VisionMissionControllerHandle handle = vision_mission_controller_create();
  assert(handle != nullptr);
  VisionPerceptionDetection detections[3]{};
  detections[0].detection = {40.0, 79.0, 20.0, 20.0, 0.90, 0};
  detections[1].detection = {40.0, 39.0, 20.0, 20.0, 0.90, 0};
  detections[2].detection = {30.0, 20.0, 40.0, 20.0, 0.90, 3};
  detections[2].center_depth_valid = 1;
  detections[2].center_depth_m = 0.70;
  detections[2].left_depth_valid = 1;
  detections[2].left_depth_m = 0.65;
  detections[2].right_depth_valid = 1;
  detections[2].right_depth_m = 0.75;

  const VisionMissionFrameResult result =
      vision_mission_controller_step_perception_v2(
          handle, detections, 3, 100.0, 100.0, 50.0, 50.0,
          1, 1, 0.1, 0.0, 0.0, 0.0, 100, 100, 0.0,
          static_cast<int>(CameraMode::kForward), 1, 1, 0, 0, 0, 0);
  assert(result.raw_line_count == 2);
  assert(result.rectified_line_count == 2);
  assert(result.imu_rectification_applied == 1);
  assert(result.targets.backboard.valid == 1);
  assert(result.goal_pose.valid == 1);
  assert(std::isfinite(result.command.pre_p2p_vx));
  assert(std::isfinite(result.command.pre_p2p_vy));
  assert(std::isfinite(result.command.pre_p2p_wz));
  assert(result.command.action_execution_kind >=
         VISION_ACTION_EXECUTION_NONE);
  assert(result.command.action_execution_kind <=
         VISION_ACTION_EXECUTION_STATIONARY);

  vision_mission_controller_reset(handle);
  vision_mission_controller_destroy(handle);
}

void TestAssociationUsesPreviousIdentityAndMatchingDepth() {
  MissionController controller(Config());
  auto first = Frame();
  auto initial = controller.StepPerception(first);
  assert(initial.perception.targets.backboard);
  assert(initial.perception.targets.backboard->center_px.u == 50.0);

  auto second = Frame();
  second.detections.erase(second.detections.begin() + 3);
  auto nearby = Observation(3, 0.61, {32.0, 20.0, 40.0, 20.0});
  nearby.center_depth_m = 1.2;
  nearby.left_depth_m = 1.1;
  nearby.right_depth_m = 1.3;
  auto high_confidence_far = Observation(3, 0.99, {75.0, 20.0, 20.0, 20.0});
  high_confidence_far.center_depth_m = 2.0;
  high_confidence_far.left_depth_m = 1.9;
  high_confidence_far.right_depth_m = 2.1;
  second.detections.push_back(nearby);
  second.detections.push_back(high_confidence_far);

  const auto associated = controller.StepPerception(second);
  assert(associated.perception.targets.backboard);
  assert(associated.perception.targets.backboard->confidence == 0.61);
  assert(associated.perception.backboard_center_depth_m == 1.2);
}

void TestMissingFrameIsNotForwardedOrCountedAsBallHit() {
  auto config = Config();
  config.enable_ball = true;
  config.ball.upper_acquire_v_norm = 1.01;
  config.ball.stable_window = 10;
  config.ball.stable_min_hits = 7;
  config.ball.tilt_down_min_hits = 99;
  MissionController controller(config);

  PerceptionFrameInput input;
  input.image_width = 100;
  input.image_height = 100;
  input.camera_feedback = {CameraMode::kForward, true};
  for (int frame = 0; frame < 6; ++frame) {
    input.now_sec = 0.02 * frame;
    input.detections = {Observation(1, 0.80, {40.0, 30.0, 20.0, 20.0})};
    const auto result = controller.StepPerception(input);
    assert(result.mission.active_mission == MissionType::kLine);
  }

  input.now_sec = 0.12;
  input.detections.clear();
  const auto missing = controller.StepPerception(input);
  assert(!missing.perception.targets.ball);
  assert(missing.mission.active_mission == MissionType::kLine);

  input.now_sec = 0.14;
  input.detections = {Observation(1, 0.61, {42.0, 30.0, 20.0, 20.0})};
  const auto reappeared = controller.StepPerception(input);
  assert(reappeared.perception.targets.ball);
  assert(reappeared.mission.active_mission == MissionType::kBall);
}

void TestGoalRemainsOutsideAssociationTracker() {
  MissionController controller(Config());
  PerceptionFrameInput input;
  input.image_width = 100;
  input.image_height = 100;
  input.camera_feedback = {CameraMode::kForward, true};
  input.detections = {
      Observation(2, 0.70, {5.0, 20.0, 20.0, 20.0}),
      Observation(2, 0.90, {75.0, 20.0, 20.0, 20.0}),
  };
  auto result = controller.StepPerception(input);
  assert(result.perception.targets.goal->center_px.u == 85.0);

  input.detections[0].detection.confidence = 0.95;
  input.detections[1].detection.confidence = 0.65;
  result = controller.StepPerception(input);
  // goal은 이전 위치와 association하지 않고 기존 최고-confidence 선택을 유지한다.
  assert(result.perception.targets.goal->center_px.u == 15.0);
}

void TestBallBackboardAndHurdleTrackersKeepSeparateState() {
  MissionController controller(Config());
  PerceptionFrameInput input;
  input.image_width = 100;
  input.image_height = 100;
  input.camera_feedback = {CameraMode::kForward, true};

  auto backboard = Observation(3, 0.80, {40.0, 15.0, 20.0, 20.0});
  backboard.center_depth_m = 1.0;
  input.detections = {
      Observation(1, 0.80, {5.0, 30.0, 20.0, 20.0}),
      backboard,
      Observation(4, 0.80, {75.0, 30.0, 20.0, 20.0}),
  };
  auto result = controller.StepPerception(input);
  assert(result.perception.targets.ball->center_px.u == 15.0);
  assert(result.perception.targets.backboard->center_px.u == 50.0);
  assert(result.perception.targets.hurdle->center_px.u == 85.0);

  auto nearby_backboard = Observation(3, 0.61, {42.0, 15.0, 20.0, 20.0});
  nearby_backboard.center_depth_m = 1.1;
  auto far_backboard = Observation(3, 0.99, {5.0, 15.0, 20.0, 20.0});
  far_backboard.center_depth_m = 1.2;
  input.detections = {
      Observation(1, 0.61, {7.0, 30.0, 20.0, 20.0}),
      Observation(1, 0.99, {70.0, 30.0, 20.0, 20.0}),
      nearby_backboard,
      far_backboard,
      Observation(4, 0.61, {73.0, 30.0, 20.0, 20.0}),
      Observation(4, 0.99, {5.0, 30.0, 20.0, 20.0}),
  };
  result = controller.StepPerception(input);
  assert(result.perception.targets.ball->center_px.u == 17.0);
  assert(result.perception.targets.backboard->center_px.u == 52.0);
  assert(result.perception.targets.hurdle->center_px.u == 83.0);
}

} // namespace

int main() {
  TestRawExtractionAndGoalDepth();
  TestPerceptionUsesCameraPrincipalPointForLineCenter();
  TestBestBackboardKeepsMatchingDepth();
  TestBackboardCenterDepthRejectsHigherConfidenceFalsePositive();
  TestImuCanBeSkippedOrApplied();
  TestEmptyDetectionFrameStillSteps();
  TestMissionPerceptionCapiUsesSharedPipeline();
  TestAssociationUsesPreviousIdentityAndMatchingDepth();
  TestMissingFrameIsNotForwardedOrCountedAsBallHit();
  TestGoalRemainsOutsideAssociationTracker();
  TestBallBackboardAndHurdleTrackersKeepSeparateState();
  std::cout << "perception pipeline tests passed\n";
  return 0;
}
