// 처리 순서: [외부 연결층] 계산 단계 자체가 아니라 Python/G1이 각 단계를 호출하는 입구다.
// 역할: C 자료형을 C++ 공통 자료형으로 변환하고, 해당 cpp의 공개 함수를 호출한 뒤 다시 변환한다.
// 값 전달: Python ctypes -> c_api 함수 -> 각 C++ 모듈 -> C 결과 구조체 순으로 왕복한다.
// 참고: 실제 ROS2 vision처럼 C++ API를 직접 쓰는 호출자는 이 파일을 거치지 않아도 된다.

#include "vision_core/c_api.h"

#include "vision_core/coordinate_rectifier.hpp"
#include "vision_core/ball_controller.hpp"
#include "vision_core/control_command.hpp"
#include "vision_core/config_loader.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_velocity_controller.hpp"
#include "vision_core/mission_controller.hpp"
#include "vision_core/motion_command_selector.hpp"
#include "vision_core/object_target_extractor.hpp"

#include <vector>

namespace {
vision_core::Features ToCore(VisionLineFeatures value) {
  return {value.u_err_near, value.u_err_lookahead, value.u_err_ctrl,
          value.slope, value.n_visible, value.in_recovery,
          value.vx_prev, value.wz_prev};
}
VisionLineFeatures FromCore(const vision_core::Features &value) {
  return {value.u_err_near, value.u_err_lookahead, value.u_err_ctrl,
          value.slope, value.n_visible, value.in_recovery,
          value.vx_prev, value.wz_prev};
}
std::vector<vision_core::Point2> CopyPoints(const VisionLinePoint *points,
                                            int count) {
  std::vector<vision_core::Point2> output;
  if (points == nullptr || count <= 0) return output;
  output.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) output.push_back({points[i].u, points[i].v});
  return output;
}

VisionObjectTarget FromCoreTarget(
    const std::optional<vision_core::ObjectTarget> &target) {
  if (!target) return {};
  return {1,
          target->class_id,
          target->confidence,
          target->box_px.x,
          target->box_px.y,
          target->box_px.width,
          target->box_px.height,
          target->center_px.u,
          target->center_px.v,
          target->rectified_center_px.u,
          target->rectified_center_px.v,
          target->center_rectified ? 1 : 0};
}

std::optional<vision_core::ObjectTarget> ToCoreTarget(
    VisionObjectTarget target) {
  if (target.valid == 0) return std::nullopt;
  vision_core::ObjectTarget output;
  output.class_id = target.class_id;
  output.confidence = target.confidence;
  output.box_px = {target.x, target.y, target.width, target.height};
  output.center_px = {target.center_u, target.center_v};
  output.rectified_center_px = {target.rectified_u, target.rectified_v};
  output.width_px = target.width;
  output.height_px = target.height;
  output.area_px = target.width * target.height;
  output.center_rectified = target.center_rectified != 0;
  return output;
}

VisionBallResult FromCoreBallResult(const vision_core::BallResult &result) {
  // 기존 G1 C API는 NONE을 표현할 수 없는 bool 필드를 사용한다. ABI를 유지하기
  // 위해 카메라가 아래에 머무는 상태까지 기존 DOWN 레벨 신호로 변환한다.
  const bool legacy_camera_down =
      result.camera_request == vision_core::CameraRequest::kDown ||
      result.mode == vision_core::BallMode::kFineAdjustForPickup ||
      result.mode == vision_core::BallMode::kPickupBall ||
      result.mode == vision_core::BallMode::kVerifyPickup ||
      result.mode == vision_core::BallMode::kVerifyPickupObservation ||
      result.mode == vision_core::BallMode::kStandUpAfterPickup;
  return {result.active ? 1 : 0,
          result.reached_pickup_pose ? 1 : 0,
          legacy_camera_down ? 1 : 0,
          static_cast<int>(result.mode),
          result.tracked.stable ? 1 : 0,
          result.tracked.visible ? 1 : 0,
          result.tracked.u_norm,
          result.tracked.v_norm,
          result.tracked.h_norm,
          result.tracked.area_norm,
          result.tracked.confidence,
          result.command.vx,
          result.command.vy,
          result.command.wz};
}

VisionHurdleResult FromCoreHurdleResult(
    const vision_core::HurdleResult &result) {
  return {result.active ? 1 : 0,
          static_cast<int>(result.camera_request),
          static_cast<int>(result.action_request),
          static_cast<int>(result.mode),
          result.tracked.stable ? 1 : 0,
          result.tracked.visible ? 1 : 0,
          result.tracked.u_norm,
          result.tracked.v_norm,
          result.tracked.h_norm,
          result.tracked.bottom_norm,
          result.tracked.confidence,
          result.command.vx,
          result.command.vy,
          result.command.wz};
}

VisionGoalResult FromCoreGoalResult(const vision_core::GoalResult &result) {
  return {result.active ? 1 : 0,
          static_cast<int>(result.camera_request),
          static_cast<int>(result.action_request),
          static_cast<int>(result.mode),
          result.tracked.stable ? 1 : 0,
          result.tracked.visible ? 1 : 0,
          result.tracked.u_norm,
          result.tracked.v_norm,
          result.tracked.h_norm,
          result.tracked.confidence,
          result.command.vx,
          result.command.vy,
          result.command.wz};
}

VisionControlCommandV3 FromCoreControlCommandV3(
    const vision_core::ControlCommand &command) {
  return {static_cast<int>(command.command_type),
          static_cast<int>(command.mission),
          command.mission_phase,
          static_cast<int>(command.control_phase),
          command.velocity.vx,
          command.velocity.vy,
          command.velocity.wz,
          command.pre_p2p_motion.vx,
          command.pre_p2p_motion.vy,
          command.pre_p2p_motion.wz,
          static_cast<int>(command.action),
          command.action_id,
          static_cast<int>(command.action_category),
          static_cast<int>(command.action_execution_kind),
          command.action_yaw_rad,
          static_cast<int>(command.camera_request)};
}

vision_core::CameraFeedback ToCameraFeedback(int camera_actual_mode,
                                             int camera_settled) {
  vision_core::CameraMode mode = vision_core::CameraMode::kTransition;
  if (camera_actual_mode ==
      static_cast<int>(vision_core::CameraMode::kForward)) {
    mode = vision_core::CameraMode::kForward;
  } else if (camera_actual_mode ==
             static_cast<int>(vision_core::CameraMode::kDown)) {
    mode = vision_core::CameraMode::kDown;
  } else if (camera_actual_mode ==
             static_cast<int>(vision_core::CameraMode::kGoal)) {
    mode = vision_core::CameraMode::kGoal;
  }
  return {mode, camera_settled != 0};
}

vision_core::ActionExecutionFeedback ToActionFeedback(
    int enabled, int done, int active) {
  return {enabled != 0, done != 0, active != 0};
}

vision_core::ControlCommand ComputeControlCommand(
    VisionControlCommandCoordinatorHandle handle,
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, int ball_has_ball,
    double line_vx, double line_vy, double line_wz,
    uint64_t feedback_action_id, int action_acknowledged, int action_done) {
  auto *coordinator =
      static_cast<vision_core::ControlCommandCoordinator *>(handle);
  if (coordinator == nullptr) return {};

  vision_core::BallResult core_ball;
  core_ball.active = ball_result.active != 0;
  core_ball.has_ball = ball_has_ball != 0;
  core_ball.mode = static_cast<vision_core::BallMode>(ball_result.mode);
  core_ball.command = {ball_result.vx, ball_result.vy, ball_result.wz};
  if (core_ball.mode == vision_core::BallMode::kFineAdjustForPickup) {
    core_ball.action_request =
        vision_core::BallActionRequest::kFineAdjustForward;
  } else if (core_ball.mode == vision_core::BallMode::kPickupBall) {
    core_ball.action_request = vision_core::BallActionRequest::kPickup;
  } else if (core_ball.mode == vision_core::BallMode::kStandUpAfterPickup) {
    core_ball.action_request = vision_core::BallActionRequest::kStandUp;
  } else if (core_ball.mode == vision_core::BallMode::kVerifyPickup) {
    core_ball.action_request = vision_core::BallActionRequest::kVerifyPickup;
  }
  if (core_ball.mode == vision_core::BallMode::kTiltCameraDownAndApproach ||
      core_ball.mode == vision_core::BallMode::kBallRecoveryDown) {
    core_ball.camera_request = vision_core::CameraRequest::kDown;
  } else if (core_ball.mode == vision_core::BallMode::kReturnCameraToLine) {
    core_ball.camera_request = vision_core::CameraRequest::kForward;
  }

  vision_core::HurdleResult core_hurdle;
  core_hurdle.active = hurdle_result.active != 0;
  core_hurdle.mode =
      static_cast<vision_core::HurdleMode>(hurdle_result.mode);
  core_hurdle.action_request =
      static_cast<vision_core::HurdleActionRequest>(
          hurdle_result.action_request);
  core_hurdle.camera_request = static_cast<vision_core::CameraRequest>(
      hurdle_result.camera_request);
  core_hurdle.command =
      {hurdle_result.vx, hurdle_result.vy, hurdle_result.wz};

  vision_core::GoalResult core_goal;
  core_goal.active = goal_result.active != 0;
  core_goal.mode = static_cast<vision_core::GoalMode>(goal_result.mode);
  core_goal.action_request = static_cast<vision_core::GoalActionRequest>(
      goal_result.action_request);
  core_goal.camera_request =
      static_cast<vision_core::CameraRequest>(goal_result.camera_request);
  core_goal.command = {goal_result.vx, goal_result.vy, goal_result.wz};

  return coordinator->Compute(
      core_ball, core_hurdle, core_goal, {line_vx, line_vy, line_wz},
      {feedback_action_id, action_acknowledged != 0, action_done != 0});
}
} // namespace

extern "C" {

VisionLineFeatureConfig vision_line_default_feature_config(void) {
  const vision_core::FeatureConfig cfg =
      vision_core::LoadDefaultAlgorithmConfig().line_features;
  return {cfg.max_centers, cfg.image_center_u, cfg.lookahead_delta_v_px,
          cfg.lookahead_alpha_normal, cfg.lookahead_alpha_recovery,
          cfg.recover_enter_nvis, cfg.recover_exit_nvis,
          cfg.recover_enter_u, cfg.recover_exit_u};
}

int vision_line_rectify_points(const VisionLinePoint *input, int count,
                               double fx, double fy, double cx, double cy,
                               double roll_rad, double pitch_rad,
                               VisionLinePoint *output) {
  if (count < 0 || (count > 0 && (input == nullptr || output == nullptr))) return -1;
  const auto result = vision_core::RectifyPixelPoints(
      CopyPoints(input, count), {fx, fy, cx, cy}, roll_rad, pitch_rad);
  for (std::size_t i = 0; i < result.size(); ++i) {
    output[i] = {result[i].u, result[i].v};
  }
  return static_cast<int>(result.size());
}

VisionLineFeatures vision_line_compute_features(
    const VisionLinePoint *points, int count, int image_width, int image_height,
    int previous_in_recovery, double vx_prev, double wz_prev,
    VisionLineFeatureConfig config) {
  vision_core::FeatureConfig cfg =
      vision_core::LoadDefaultAlgorithmConfig().line_features;
  cfg.max_centers = config.max_centers;
  cfg.image_center_u = config.image_center_u;
  cfg.lookahead_delta_v_px = config.lookahead_delta_v_px;
  cfg.lookahead_alpha_normal = config.lookahead_alpha_normal;
  cfg.lookahead_alpha_recovery = config.lookahead_alpha_recovery;
  cfg.recover_enter_nvis = config.recover_enter_nvis;
  cfg.recover_exit_nvis = config.recover_exit_nvis;
  cfg.recover_enter_u = config.recover_enter_u;
  cfg.recover_exit_u = config.recover_exit_u;
  return FromCore(vision_core::ComputeLineFeatures(
      CopyPoints(points, count), image_width, image_height,
      previous_in_recovery != 0, vx_prev, wz_prev, cfg));
}

VisionLineFeatures vision_line_compute_features_v2(
    const VisionLinePoint *points, int count, int image_width, int image_height,
    int previous_in_recovery, double vx_prev, double wz_prev,
    VisionLineFeatureConfig config, VisionLineFeatureState *state) {
  vision_core::FeatureConfig cfg =
      vision_core::LoadDefaultAlgorithmConfig().line_features;
  cfg.max_centers = config.max_centers;
  cfg.image_center_u = config.image_center_u;
  cfg.lookahead_delta_v_px = config.lookahead_delta_v_px;
  cfg.lookahead_alpha_normal = config.lookahead_alpha_normal;
  cfg.lookahead_alpha_recovery = config.lookahead_alpha_recovery;
  cfg.recover_enter_nvis = config.recover_enter_nvis;
  cfg.recover_exit_nvis = config.recover_exit_nvis;
  cfg.recover_enter_u = config.recover_enter_u;
  cfg.recover_exit_u = config.recover_exit_u;

  vision_core::LineFeatureState core_state;
  vision_core::LineFeatureState *core_state_ptr = nullptr;
  if (state != nullptr) {
    core_state.filtered_curve_score = state->filtered_curve_score;
    core_state.initialized = state->initialized != 0;
    core_state_ptr = &core_state;
  }
  const auto result = vision_core::ComputeLineFeatures(
      CopyPoints(points, count), image_width, image_height,
      previous_in_recovery != 0, vx_prev, wz_prev, cfg, core_state_ptr);
  if (state != nullptr) {
    state->filtered_curve_score = core_state.filtered_curve_score;
    state->initialized = core_state.initialized ? 1 : 0;
  }
  return FromCore(result);
}

void vision_line_feature_state_reset(VisionLineFeatureState *state) {
  if (state == nullptr) return;
  state->filtered_curve_score = 0.0;
  state->initialized = 0;
}

VisionLineControllerHandle vision_line_controller_create(double observation_dt) {
  const auto config = vision_core::LoadDefaultAlgorithmConfig();
  return new vision_core::LineVelocityController(config.line, observation_dt);
}
void vision_line_controller_destroy(VisionLineControllerHandle handle) {
  delete static_cast<vision_core::LineVelocityController *>(handle);
}
int vision_line_controller_set_path(VisionLineControllerHandle handle,
                                    const VisionLinePoint *path_xy,
                                    const double *path_s,
                                    const double *path_heading, int count) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller == nullptr || count <= 0 || path_xy == nullptr || path_s == nullptr ||
      path_heading == nullptr) return -1;
  controller->SetPath(CopyPoints(path_xy, count),
                      std::vector<double>(path_s, path_s + count),
                      std::vector<double>(path_heading, path_heading + count));
  return 0;
}
void vision_line_controller_observe(VisionLineControllerHandle handle,
                                    VisionLineFeatures features, double x,
                                    double y, double yaw) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller != nullptr) controller->Observe(ToCore(features), {x, y, yaw});
}
void vision_line_controller_compute(VisionLineControllerHandle handle,
                                    VisionLineFeatures features, double *vx,
                                    double *wz) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller == nullptr || vx == nullptr || wz == nullptr) return;
  const auto command = controller->Compute(ToCore(features));
  *vx = command.vx;
  *wz = command.wz;
}
void vision_line_controller_compute_tracking(VisionLineControllerHandle handle,
                                             VisionLineFeatures features,
                                             double *vx, double *wz) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller == nullptr || vx == nullptr || wz == nullptr) return;
  const auto command = controller->ComputeTracking(ToCore(features));
  *vx = command.vx;
  *wz = command.wz;
}
void vision_line_controller_compute_search_rotation(
    VisionLineControllerHandle handle, double *vx, double *wz) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller == nullptr || vx == nullptr || wz == nullptr) return;
  const auto command = controller->ComputeSearchRotation();
  *vx = command.vx;
  *wz = command.wz;
}
void vision_line_controller_compute_lookahead_approach(
    VisionLineControllerHandle handle, VisionLineFeatures features, double *vx,
    double *wz) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller == nullptr || vx == nullptr || wz == nullptr) return;
  const auto command = controller->ComputeLookaheadApproach(ToCore(features));
  *vx = command.vx;
  *wz = command.wz;
}
int vision_line_controller_in_recovery(VisionLineControllerHandle handle) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  return controller != nullptr && controller->InRecovery() ? 1 : 0;
}
void vision_line_controller_begin_reacquisition(
    VisionLineControllerHandle handle) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller != nullptr) controller->BeginReacquisition();
}
void vision_line_controller_reset(VisionLineControllerHandle handle) {
  auto *controller = static_cast<vision_core::LineVelocityController *>(handle);
  if (controller != nullptr) controller->Reset();
}

VisionObjectTargets vision_object_extract_targets(
    const VisionObjectDetection *detections, int count,
    int ball_class_id, int goal_class_id, int backboard_class_id,
    int hurdle_class_id, double confidence_threshold) {
  std::vector<vision_core::Detection> input;
  if (detections != nullptr && count > 0) {
    input.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
      input.push_back({
          {detections[i].x, detections[i].y, detections[i].width,
           detections[i].height},
          detections[i].confidence,
          detections[i].class_id});
    }
  }
  vision_core::ObjectTargetConfig config =
      vision_core::LoadDefaultAlgorithmConfig().object_targets;
  config.ball_class_id = ball_class_id;
  config.goal_class_id = goal_class_id;
  config.backboard_class_id = backboard_class_id;
  config.hurdle_class_id = hurdle_class_id;
  config.ball_confidence = confidence_threshold;
  config.goal_confidence = confidence_threshold;
  config.backboard_confidence = confidence_threshold;
  config.hurdle_confidence = confidence_threshold;
  const auto targets = vision_core::ExtractObjectTargets(input, config);
  return {FromCoreTarget(targets.ball), FromCoreTarget(targets.goal),
          FromCoreTarget(targets.backboard), FromCoreTarget(targets.hurdle)};
}

VisionBallControllerHandle vision_ball_controller_create(void) {
  return new vision_core::BallController();
}

void vision_ball_controller_destroy(VisionBallControllerHandle handle) {
  delete static_cast<vision_core::BallController *>(handle);
}

VisionBallResult vision_ball_controller_compute(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  if (controller == nullptr) return {};
  const auto result = controller->Compute(
      ToCoreTarget(ball_target), image_width, image_height, now_sec);
  return FromCoreBallResult(result);
}

VisionBallResult vision_ball_controller_compute_v2(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int camera_actual_mode, int camera_settled) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  if (controller == nullptr) return {};
  const auto result = controller->Compute(
      ToCoreTarget(ball_target), image_width, image_height, now_sec, line_vx,
      ToCameraFeedback(camera_actual_mode, camera_settled));
  return FromCoreBallResult(result);
}

VisionBallResult vision_ball_controller_compute_v3(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  if (controller == nullptr) return {};
  const auto result = controller->Compute(
      ToCoreTarget(ball_target), image_width, image_height, now_sec, line_vx,
      line_reference_valid != 0,
      ToCameraFeedback(camera_actual_mode, camera_settled));
  return FromCoreBallResult(result);
}

VisionBallResult vision_ball_controller_compute_v4(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled,
    int action_feedback_enabled, int action_done, int action_active) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  if (controller == nullptr) return {};
  return FromCoreBallResult(controller->Compute(
      ToCoreTarget(ball_target), image_width, image_height, now_sec, line_vx,
      line_reference_valid != 0,
      ToCameraFeedback(camera_actual_mode, camera_settled),
      ToActionFeedback(action_feedback_enabled, action_done, action_active)));
}

void vision_ball_controller_reset(VisionBallControllerHandle handle) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  if (controller != nullptr) controller->Reset();
}

int vision_ball_controller_has_ball(VisionBallControllerHandle handle) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  return controller != nullptr && controller->HasBall() ? 1 : 0;
}

int vision_ball_controller_pickup_failed(VisionBallControllerHandle handle) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  return controller != nullptr && controller->PickupFailed() ? 1 : 0;
}

void vision_ball_controller_set_has_ball(VisionBallControllerHandle handle,
                                         int has_ball) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  if (controller != nullptr) controller->SetHasBall(has_ball != 0);
}

int vision_ball_controller_pickup_attempt_count(
    VisionBallControllerHandle handle) {
  auto *controller = static_cast<vision_core::BallController *>(handle);
  return controller != nullptr ? controller->PickupAttemptCount() : 0;
}

VisionHurdleControllerHandle vision_hurdle_controller_create(void) {
  return new vision_core::HurdleController();
}

void vision_hurdle_controller_destroy(VisionHurdleControllerHandle handle) {
  delete static_cast<vision_core::HurdleController *>(handle);
}

VisionHurdleResult vision_hurdle_controller_compute(
    VisionHurdleControllerHandle handle, VisionObjectTarget hurdle_target,
    int image_width, int image_height, double now_sec,
    int camera_actual_mode, int camera_settled) {
  auto *controller = static_cast<vision_core::HurdleController *>(handle);
  if (controller == nullptr) return {};
  return FromCoreHurdleResult(controller->Compute(
      ToCoreTarget(hurdle_target), image_width, image_height, now_sec,
      ToCameraFeedback(camera_actual_mode, camera_settled)));
}

VisionHurdleResult vision_hurdle_controller_compute_v2(
    VisionHurdleControllerHandle handle, VisionObjectTarget hurdle_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled) {
  auto *controller = static_cast<vision_core::HurdleController *>(handle);
  if (controller == nullptr) return {};
  return FromCoreHurdleResult(controller->Compute(
      ToCoreTarget(hurdle_target), image_width, image_height, now_sec, line_vx,
      line_reference_valid != 0,
      ToCameraFeedback(camera_actual_mode, camera_settled)));
}

VisionHurdleResult vision_hurdle_controller_compute_v3(
    VisionHurdleControllerHandle handle, VisionObjectTarget hurdle_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled,
    int action_feedback_enabled, int action_done, int action_active) {
  auto *controller = static_cast<vision_core::HurdleController *>(handle);
  if (controller == nullptr) return {};
  return FromCoreHurdleResult(controller->Compute(
      ToCoreTarget(hurdle_target), image_width, image_height, now_sec, line_vx,
      line_reference_valid != 0,
      ToCameraFeedback(camera_actual_mode, camera_settled),
      ToActionFeedback(action_feedback_enabled, action_done, action_active)));
}

void vision_hurdle_controller_reset(VisionHurdleControllerHandle handle) {
  auto *controller = static_cast<vision_core::HurdleController *>(handle);
  if (controller != nullptr) controller->Reset();
}

VisionGoalControllerHandle vision_goal_controller_create(void) {
  return new vision_core::GoalController();
}

VisionGoalPoseObservation vision_goal_pose_from_edge_depths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, double fx, double fy, double cx, double cy,
    double confidence) {
  const auto pose = vision_core::EstimateGoalPoseFromEdgeDepths(
      left_u_px, left_depth_m, right_u_px, right_depth_m,
      vision_core::Intrinsics{fx, fy, cx, cy}, confidence);
  return {pose.valid ? 1 : 0, pose.x_m, pose.z_m, pose.yaw_rad,
          pose.confidence};
}

void vision_goal_controller_destroy(VisionGoalControllerHandle handle) {
  delete static_cast<vision_core::GoalController *>(handle);
}

void vision_goal_controller_start_after_pickup(
    VisionGoalControllerHandle handle, double now_sec) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  if (controller != nullptr) controller->StartAfterPickup(now_sec);
}

void vision_goal_controller_set_has_ball(VisionGoalControllerHandle handle,
                                         int has_ball) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  if (controller != nullptr) controller->SetHasBall(has_ball != 0);
}

void vision_goal_controller_update_ball_state(
    VisionGoalControllerHandle handle, int has_ball, int ball_mode) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  if (controller == nullptr) return;
  vision_core::BallResult ball_result;
  ball_result.has_ball = has_ball != 0;
  ball_result.mode = static_cast<vision_core::BallMode>(ball_mode);
  controller->UpdateBallState(ball_result);
}

int vision_goal_controller_has_ball(VisionGoalControllerHandle handle) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  return controller != nullptr && controller->HasBall() ? 1 : 0;
}

VisionGoalResult vision_goal_controller_compute(
    VisionGoalControllerHandle handle, VisionObjectTarget goal_target,
    int image_width, int image_height, double now_sec,
    int line_reference_valid, int camera_actual_mode, int camera_settled) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  if (controller == nullptr) return {};
  return FromCoreGoalResult(controller->Compute(
      ToCoreTarget(goal_target), image_width, image_height, now_sec,
      line_reference_valid != 0,
      ToCameraFeedback(camera_actual_mode, camera_settled)));
}

VisionGoalResult vision_goal_controller_compute_v2(
    VisionGoalControllerHandle handle, VisionObjectTarget goal_target,
    VisionObjectTarget backboard_target, VisionGoalPoseObservation goal_pose,
    int image_width, int image_height, double now_sec,
    int line_reference_valid, int camera_actual_mode, int camera_settled) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  if (controller == nullptr) return {};
  const vision_core::GoalPoseObservation core_pose{
      goal_pose.valid != 0, goal_pose.x_m, goal_pose.z_m,
      goal_pose.yaw_rad, goal_pose.confidence};
  return FromCoreGoalResult(controller->Compute(
      ToCoreTarget(goal_target), ToCoreTarget(backboard_target), core_pose,
      image_width, image_height, now_sec, line_reference_valid != 0,
      ToCameraFeedback(camera_actual_mode, camera_settled)));
}

VisionGoalResult vision_goal_controller_compute_v3(
    VisionGoalControllerHandle handle, VisionObjectTarget goal_target,
    VisionObjectTarget backboard_target, VisionGoalPoseObservation goal_pose,
    int image_width, int image_height, double now_sec,
    int line_reference_valid, int camera_actual_mode, int camera_settled,
    int action_feedback_enabled, int action_done, int action_active) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  if (controller == nullptr) return {};
  const vision_core::GoalPoseObservation core_pose{
      goal_pose.valid != 0, goal_pose.x_m, goal_pose.z_m,
      goal_pose.yaw_rad, goal_pose.confidence};
  return FromCoreGoalResult(controller->Compute(
      ToCoreTarget(goal_target), ToCoreTarget(backboard_target), core_pose,
      image_width, image_height, now_sec, line_reference_valid != 0,
      ToCameraFeedback(camera_actual_mode, camera_settled),
      ToActionFeedback(action_feedback_enabled, action_done, action_active)));
}

void vision_goal_controller_reset(VisionGoalControllerHandle handle) {
  auto *controller = static_cast<vision_core::GoalController *>(handle);
  if (controller != nullptr) controller->Reset();
}

VisionSelectedMotionCommand vision_select_motion_command(
    VisionLineControllerHandle line_controller, VisionLineFeatures line_features,
    VisionBallResult ball_result) {
  auto *controller =
      static_cast<vision_core::LineVelocityController *>(line_controller);
  if (controller == nullptr) return {};

  vision_core::BallResult core_ball;
  core_ball.active = ball_result.active != 0;
  core_ball.mode = static_cast<vision_core::BallMode>(ball_result.mode);
  core_ball.command = {ball_result.vx, ball_result.vy, ball_result.wz};
  const auto selected = vision_core::SelectMotionCommand(
      core_ball, ToCore(line_features), *controller);
  return {selected.command.vx,
          selected.command.vy,
          selected.command.wz,
          static_cast<int>(selected.source),
          static_cast<int>(selected.ball_mode)};
}

VisionSelectedMotionCommand vision_select_motion_command_v2(
    VisionBallResult ball_result, double line_vx, double line_vy,
    double line_wz) {
  vision_core::BallResult core_ball;
  core_ball.active = ball_result.active != 0;
  core_ball.mode = static_cast<vision_core::BallMode>(ball_result.mode);
  core_ball.command = {ball_result.vx, ball_result.vy, ball_result.wz};
  const auto selected = vision_core::SelectMotionCommand(
      core_ball, {line_vx, line_vy, line_wz});
  return {selected.command.vx,
          selected.command.vy,
          selected.command.wz,
          static_cast<int>(selected.source),
          static_cast<int>(selected.ball_mode)};
}

VisionSelectedMissionCommand vision_select_mission_command(
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, double line_vx, double line_vy,
    double line_wz) {
  return vision_select_mission_command_v2(
      ball_result, hurdle_result, goal_result, 0,
      line_vx, line_vy, line_wz);
}

VisionSelectedMissionCommand vision_select_mission_command_v2(
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, int ball_has_ball,
    double line_vx, double line_vy, double line_wz) {
  vision_core::BallResult core_ball;
  core_ball.active = ball_result.active != 0;
  core_ball.has_ball = ball_has_ball != 0;
  core_ball.mode = static_cast<vision_core::BallMode>(ball_result.mode);
  core_ball.command = {ball_result.vx, ball_result.vy, ball_result.wz};

  vision_core::HurdleResult core_hurdle;
  core_hurdle.active = hurdle_result.active != 0;
  core_hurdle.mode =
      static_cast<vision_core::HurdleMode>(hurdle_result.mode);
  core_hurdle.command =
      {hurdle_result.vx, hurdle_result.vy, hurdle_result.wz};

  vision_core::GoalResult core_goal;
  core_goal.active = goal_result.active != 0;
  core_goal.mode = static_cast<vision_core::GoalMode>(goal_result.mode);
  core_goal.command = {goal_result.vx, goal_result.vy, goal_result.wz};

  const auto selected = vision_core::SelectMotionCommand(
      core_ball, core_hurdle, core_goal, {line_vx, line_vy, line_wz});
  int active_mode = 0;
  if (selected.source == vision_core::CommandSource::kBall) {
    active_mode = static_cast<int>(selected.ball_mode);
  } else if (selected.source == vision_core::CommandSource::kHurdle) {
    active_mode = static_cast<int>(selected.hurdle_mode);
  } else if (selected.source == vision_core::CommandSource::kGoal) {
    active_mode = static_cast<int>(selected.goal_mode);
  }
  return {selected.command.vx,
          selected.command.vy,
          selected.command.wz,
          static_cast<int>(selected.source),
          active_mode};
}

VisionControlCommandCoordinatorHandle
vision_control_command_coordinator_create(void) {
  return new vision_core::ControlCommandCoordinator();
}

void vision_control_command_coordinator_destroy(
    VisionControlCommandCoordinatorHandle handle) {
  delete static_cast<vision_core::ControlCommandCoordinator *>(handle);
}

VisionControlCommand vision_control_command_compute(
    VisionControlCommandCoordinatorHandle handle,
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, double line_vx, double line_vy,
    double line_wz, uint64_t feedback_action_id, int action_acknowledged,
    int action_done) {
  return vision_control_command_compute_v2(
      handle, ball_result, hurdle_result, goal_result, 0,
      line_vx, line_vy, line_wz, feedback_action_id,
      action_acknowledged, action_done);
}

VisionControlCommand vision_control_command_compute_v2(
    VisionControlCommandCoordinatorHandle handle,
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, int ball_has_ball,
    double line_vx, double line_vy, double line_wz,
    uint64_t feedback_action_id, int action_acknowledged, int action_done) {
  if (handle == nullptr) return {};
  const auto command = ComputeControlCommand(
      handle, ball_result, hurdle_result, goal_result, ball_has_ball,
      line_vx, line_vy, line_wz, feedback_action_id, action_acknowledged,
      action_done);
  return {static_cast<int>(command.command_type),
          static_cast<int>(command.mission),
          command.mission_phase,
          static_cast<int>(command.control_phase),
          command.velocity.vx,
          command.velocity.vy,
          command.velocity.wz,
          static_cast<int>(command.action),
          command.action_id,
          static_cast<int>(command.camera_request)};
}

VisionControlCommandV3 vision_control_command_compute_v3(
    VisionControlCommandCoordinatorHandle handle,
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, int ball_has_ball,
    double line_vx, double line_vy, double line_wz,
    uint64_t feedback_action_id, int action_acknowledged, int action_done) {
  if (handle == nullptr) return {};
  const auto command = ComputeControlCommand(
      handle, ball_result, hurdle_result, goal_result, ball_has_ball,
      line_vx, line_vy, line_wz, feedback_action_id, action_acknowledged,
      action_done);
  return FromCoreControlCommandV3(command);
}

void vision_control_command_coordinator_reset(
    VisionControlCommandCoordinatorHandle handle) {
  auto *coordinator =
      static_cast<vision_core::ControlCommandCoordinator *>(handle);
  if (coordinator != nullptr) coordinator->Reset();
}

VisionMissionControllerHandle vision_mission_controller_create(void) {
  return new vision_core::MissionController();
}

void vision_mission_controller_destroy(VisionMissionControllerHandle handle) {
  delete static_cast<vision_core::MissionController *>(handle);
}

VisionMissionFrameResult vision_mission_controller_step_perception_v1(
    VisionMissionControllerHandle handle,
    const VisionPerceptionDetection *detections, int detection_count,
    double fx, double fy, double cx, double cy,
    int enable_imu_rectification, int imu_valid,
    double roll_rad, double pitch_rad,
    double previous_vx, double previous_wz,
    int image_width, int image_height, double now_sec,
    int camera_actual_mode, int camera_settled,
    int command_transport_enabled,
    uint64_t feedback_action_id, int action_acknowledged, int action_done) {
  return vision_mission_controller_step_perception_v2(
      handle, detections, detection_count, fx, fy, cx, cy,
      enable_imu_rectification, imu_valid, roll_rad, pitch_rad, previous_vx,
      previous_wz, image_width, image_height, now_sec, camera_actual_mode,
      camera_settled, command_transport_enabled, feedback_action_id,
      action_acknowledged, action_done, 0);
}

VisionMissionFrameResult vision_mission_controller_step_perception_v2(
    VisionMissionControllerHandle handle,
    const VisionPerceptionDetection *detections, int detection_count,
    double fx, double fy, double cx, double cy,
    int enable_imu_rectification, int imu_valid,
    double roll_rad, double pitch_rad,
    double previous_vx, double previous_wz,
    int image_width, int image_height, double now_sec,
    int camera_actual_mode, int camera_settled,
    int command_transport_enabled,
    uint64_t feedback_action_id, int action_acknowledged, int action_done,
    int action_ready) {
  auto *controller = static_cast<vision_core::MissionController *>(handle);
  if (controller == nullptr) return {};

  vision_core::PerceptionFrameInput input;
  if (detections != nullptr && detection_count > 0) {
    input.detections.reserve(static_cast<std::size_t>(detection_count));
    for (int index = 0; index < detection_count; ++index) {
      const auto &source = detections[index];
      vision_core::PerceptionDetection observation;
      observation.detection = {
          {source.detection.x, source.detection.y,
           source.detection.width, source.detection.height},
          source.detection.confidence, source.detection.class_id};
      if (source.center_depth_valid != 0) {
        observation.center_depth_m = source.center_depth_m;
      }
      if (source.left_depth_valid != 0) {
        observation.left_depth_m = source.left_depth_m;
      }
      if (source.right_depth_valid != 0) {
        observation.right_depth_m = source.right_depth_m;
      }
      input.detections.push_back(observation);
    }
  }
  input.intrinsics = {fx, fy, cx, cy};
  input.enable_imu_rectification = enable_imu_rectification != 0;
  input.imu_valid = imu_valid != 0;
  input.roll_rad = roll_rad;
  input.pitch_rad = pitch_rad;
  input.previous_vx = previous_vx;
  input.previous_wz = previous_wz;
  input.image_width = image_width;
  input.image_height = image_height;
  input.now_sec = now_sec;
  input.camera_feedback =
      ToCameraFeedback(camera_actual_mode, camera_settled);
  input.command_transport_enabled = command_transport_enabled != 0;
  input.delivery_feedback = {
      feedback_action_id, action_acknowledged != 0, action_done != 0,
      action_ready != 0};

  const auto result = controller->StepPerception(input);
  VisionMissionFrameResult output{};
  output.command = FromCoreControlCommandV3(result.mission.command);
  output.active_mission = static_cast<int>(result.mission.active_mission);
  output.line_features = FromCore(result.mission.line_features);
  output.line_vx = result.mission.line_command.vx;
  output.line_vy = result.mission.line_command.vy;
  output.line_wz = result.mission.line_command.wz;
  output.line_computed = result.mission.line_computed ? 1 : 0;
  output.line_in_recovery = result.mission.line_in_recovery ? 1 : 0;
  output.has_ball = result.mission.has_ball ? 1 : 0;
  output.ball = FromCoreBallResult(result.mission.ball);
  output.hurdle = FromCoreHurdleResult(result.mission.hurdle);
  output.goal = FromCoreGoalResult(result.mission.goal);
  output.targets = {
      FromCoreTarget(result.perception.targets.ball),
      FromCoreTarget(result.perception.targets.goal),
      FromCoreTarget(result.perception.targets.backboard),
      FromCoreTarget(result.perception.targets.hurdle)};
  output.goal_pose = {
      result.perception.goal_pose.valid ? 1 : 0,
      result.perception.goal_pose.x_m,
      result.perception.goal_pose.z_m,
      result.perception.goal_pose.yaw_rad,
      result.perception.goal_pose.confidence};
  output.imu_rectification_applied =
      result.perception.imu_rectification_applied ? 1 : 0;
  output.raw_line_count =
      static_cast<int>(result.perception.raw_line_centers.size());
  output.rectified_line_count =
      static_cast<int>(result.perception.line_centers.size());
  return output;
}

void vision_mission_controller_reset(VisionMissionControllerHandle handle) {
  auto *controller = static_cast<vision_core::MissionController *>(handle);
  if (controller != nullptr) controller->Reset();
}

} // extern "C"
