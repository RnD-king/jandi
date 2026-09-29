#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VisionLinePoint {
  // OpenCV pixel convention: u right(+), v down(+).
  double u;
  double v;
} VisionLinePoint;

typedef struct VisionLineFeatures {
  // u errors and slope use image-right as the positive direction.
  double u_err_near;
  double u_err_lookahead;
  double u_err_ctrl;
  double slope;
  double n_visible;
  double in_recovery;
  double vx_prev;
  double wz_prev;
} VisionLineFeatures;

typedef struct VisionLineFeatureConfig {
  int max_centers;
  double image_center_u;
  double lookahead_delta_v_px;
  double lookahead_alpha_normal;
  double lookahead_alpha_recovery;
  double recover_enter_nvis;
  double recover_exit_nvis;
  double recover_enter_u;
  double recover_exit_u;
} VisionLineFeatureConfig;

typedef void *VisionLineControllerHandle;

VisionLineFeatureConfig vision_line_default_feature_config(void);
int vision_line_rectify_points(const VisionLinePoint *input, int count,
                               double fx, double fy, double cx, double cy,
                               double roll_rad, double pitch_rad,
                               VisionLinePoint *output);
VisionLineFeatures vision_line_compute_features(
    const VisionLinePoint *points, int count, int image_width, int image_height,
    int previous_in_recovery, double vx_prev, double wz_prev,
    VisionLineFeatureConfig config);

// 기존 VisionLineFeatures/Config ABI는 유지하고, 커브 평활화 상태만 별도
// 포인터로 전달하는 적응형 lookahead API다.
typedef struct VisionLineFeatureState {
  double filtered_curve_score;
  int initialized;
} VisionLineFeatureState;

VisionLineFeatures vision_line_compute_features_v2(
    const VisionLinePoint *points, int count, int image_width, int image_height,
    int previous_in_recovery, double vx_prev, double wz_prev,
    VisionLineFeatureConfig config, VisionLineFeatureState *state);
void vision_line_feature_state_reset(VisionLineFeatureState *state);
VisionLineControllerHandle vision_line_controller_create(double observation_dt);
void vision_line_controller_destroy(VisionLineControllerHandle handle);
int vision_line_controller_set_path(VisionLineControllerHandle handle,
                                    const VisionLinePoint *path_xy,
                                    const double *path_s,
                                    const double *path_heading, int count);
void vision_line_controller_observe(VisionLineControllerHandle handle,
                                    VisionLineFeatures features, double x,
                                    double y, double yaw);
void vision_line_controller_compute(VisionLineControllerHandle handle,
                                    VisionLineFeatures features, double *vx,
                                    double *wz);
void vision_line_controller_compute_tracking(VisionLineControllerHandle handle,
                                             VisionLineFeatures features,
                                             double *vx, double *wz);
void vision_line_controller_compute_search_rotation(
    VisionLineControllerHandle handle, double *vx, double *wz);
void vision_line_controller_compute_lookahead_approach(
    VisionLineControllerHandle handle, VisionLineFeatures features, double *vx,
    double *wz);
int vision_line_controller_in_recovery(VisionLineControllerHandle handle);
void vision_line_controller_begin_reacquisition(
    VisionLineControllerHandle handle);
void vision_line_controller_reset(VisionLineControllerHandle handle);

typedef struct VisionObjectDetection {
  double x;
  double y;
  double width;
  double height;
  double confidence;
  int class_id;
} VisionObjectDetection;

typedef struct VisionObjectTarget {
  int valid;
  int class_id;
  double confidence;
  double x;
  double y;
  double width;
  double height;
  double center_u;
  double center_v;
  double rectified_u;
  double rectified_v;
  int center_rectified;
} VisionObjectTarget;

typedef struct VisionObjectTargets {
  VisionObjectTarget ball;
  VisionObjectTarget goal;
  VisionObjectTarget backboard;
  VisionObjectTarget hurdle;
} VisionObjectTargets;

VisionObjectTargets vision_object_extract_targets(
    const VisionObjectDetection *detections, int count,
    int ball_class_id, int goal_class_id, int backboard_class_id,
    int hurdle_class_id, double confidence_threshold);

typedef void *VisionBallControllerHandle;

typedef struct VisionBallResult {
  int active;
  int reached_pickup_pose;
  int request_camera_down;
  int mode;
  int stable;
  int visible;
  double u_norm;
  double v_norm;
  double h_norm;
  double area_norm;
  double confidence;
  double vx;
  double vy;
  double wz;
} VisionBallResult;

VisionBallControllerHandle vision_ball_controller_create(void);
void vision_ball_controller_destroy(VisionBallControllerHandle handle);
VisionBallResult vision_ball_controller_compute(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec);
// V2 preserves VisionBallResult's ABI while adding the context required by the
// G1 adapter. camera_actual_mode: 0=FORWARD, 1=DOWN, 2=TRANSITION.
VisionBallResult vision_ball_controller_compute_v2(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int camera_actual_mode, int camera_settled);
// V3 keeps every existing struct/symbol unchanged and adds an explicit
// reference-valid bit. Set line_reference_valid=1 only for normal line
// tracking; RECOV/coast/search vx values must pass 0 even when they are > 0.
VisionBallResult vision_ball_controller_compute_v3(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled);
// V4 enables the same ACTION feedback path used by the ROS adapter.
VisionBallResult vision_ball_controller_compute_v4(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled,
    int action_feedback_enabled, int action_done, int action_active);
void vision_ball_controller_reset(VisionBallControllerHandle handle);
int vision_ball_controller_has_ball(VisionBallControllerHandle handle);
int vision_ball_controller_pickup_failed(VisionBallControllerHandle handle);
void vision_ball_controller_set_has_ball(
    VisionBallControllerHandle handle, int has_ball);
int vision_ball_controller_pickup_attempt_count(
    VisionBallControllerHandle handle);

typedef void *VisionHurdleControllerHandle;

typedef struct VisionHurdleResult {
  int active;
  int camera_request;
  int action_request;
  int mode;
  int stable;
  int visible;
  double u_norm;
  double v_norm;
  double h_norm;
  double bottom_norm;
  double confidence;
  double vx;
  double vy;
  double wz;
} VisionHurdleResult;

VisionHurdleControllerHandle vision_hurdle_controller_create(void);
void vision_hurdle_controller_destroy(VisionHurdleControllerHandle handle);
VisionHurdleResult vision_hurdle_controller_compute(
    VisionHurdleControllerHandle handle, VisionObjectTarget hurdle_target,
    int image_width, int image_height, double now_sec,
    int camera_actual_mode, int camera_settled);
VisionHurdleResult vision_hurdle_controller_compute_v2(
    VisionHurdleControllerHandle handle, VisionObjectTarget hurdle_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled);
VisionHurdleResult vision_hurdle_controller_compute_v3(
    VisionHurdleControllerHandle handle, VisionObjectTarget hurdle_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled,
    int action_feedback_enabled, int action_done, int action_active);
void vision_hurdle_controller_reset(VisionHurdleControllerHandle handle);

typedef void *VisionGoalControllerHandle;

typedef struct VisionGoalResult {
  int active;
  int camera_request;
  int action_request;
  int mode;
  int stable;
  int visible;
  double u_norm;
  double v_norm;
  double h_norm;
  double confidence;
  double vx;
  double vy;
  double wz;
} VisionGoalResult;

typedef struct VisionGoalPoseObservation {
  int valid;
  double x_m;
  double z_m;
  double yaw_rad;
  double confidence;
} VisionGoalPoseObservation;

VisionGoalPoseObservation vision_goal_pose_from_edge_depths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, double fx, double fy, double cx, double cy,
    double confidence);

VisionGoalControllerHandle vision_goal_controller_create(void);
void vision_goal_controller_destroy(VisionGoalControllerHandle handle);
void vision_goal_controller_start_after_pickup(
    VisionGoalControllerHandle handle, double now_sec);
void vision_goal_controller_set_has_ball(
    VisionGoalControllerHandle handle, int has_ball);
void vision_goal_controller_update_ball_state(
    VisionGoalControllerHandle handle, int has_ball, int ball_mode);
int vision_goal_controller_has_ball(VisionGoalControllerHandle handle);
VisionGoalResult vision_goal_controller_compute(
    VisionGoalControllerHandle handle, VisionObjectTarget goal_target,
    int image_width, int image_height, double now_sec,
    int line_reference_valid, int camera_actual_mode, int camera_settled);
// 기존 결과 구조체 ABI는 유지하면서 백보드 bbox와 RGB-D 자세를 추가한다.
VisionGoalResult vision_goal_controller_compute_v2(
    VisionGoalControllerHandle handle, VisionObjectTarget goal_target,
    VisionObjectTarget backboard_target, VisionGoalPoseObservation goal_pose,
    int image_width, int image_height, double now_sec,
    int line_reference_valid, int camera_actual_mode, int camera_settled);
VisionGoalResult vision_goal_controller_compute_v3(
    VisionGoalControllerHandle handle, VisionObjectTarget goal_target,
    VisionObjectTarget backboard_target, VisionGoalPoseObservation goal_pose,
    int image_width, int image_height, double now_sec,
    int line_reference_valid, int camera_actual_mode, int camera_settled,
    int action_feedback_enabled, int action_done, int action_active);
void vision_goal_controller_reset(VisionGoalControllerHandle handle);

typedef struct VisionSelectedMotionCommand {
  double vx;
  double vy;
  double wz;
  int source;
  int ball_mode;
} VisionSelectedMotionCommand;

VisionSelectedMotionCommand vision_select_motion_command(
    VisionLineControllerHandle line_controller, VisionLineFeatures line_features,
    VisionBallResult ball_result);
// Use this overload when line vx/vy/wz was already computed this frame. It
// performs selection only and therefore does not advance line-controller state
// a second time.
VisionSelectedMotionCommand vision_select_motion_command_v2(
    VisionBallResult ball_result, double line_vx, double line_vy,
    double line_wz);

typedef struct VisionSelectedMissionCommand {
  double vx;
  double vy;
  double wz;
  int source;
  int active_mode;
} VisionSelectedMissionCommand;

VisionSelectedMissionCommand vision_select_mission_command(
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, double line_vx, double line_vy,
    double line_wz);
VisionSelectedMissionCommand vision_select_mission_command_v2(
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, int ball_has_ball,
    double line_vx, double line_vy, double line_wz);

typedef void *VisionControlCommandCoordinatorHandle;

typedef struct VisionControlCommand {
  int command_type;
  int mission;
  int mission_phase;
  int control_phase;
  double vx;
  double vy;
  double wz;
  int action;
  uint64_t action_id;
  int camera_request;
} VisionControlCommand;

typedef enum VisionActionCategory {
  VISION_ACTION_CATEGORY_NONE = 0,
  VISION_ACTION_CATEGORY_MISSION = 1,
  VISION_ACTION_CATEGORY_LOCOMOTION = 2
} VisionActionCategory;

typedef enum VisionActionExecutionKind {
  VISION_ACTION_EXECUTION_NONE = 0,
  VISION_ACTION_EXECUTION_VELOCITY_COMPATIBLE = 1,
  VISION_ACTION_EXECUTION_DISCRETE = 2,
  VISION_ACTION_EXECUTION_STATIONARY = 3
} VisionActionExecutionKind;

// 기존 VisionControlCommand ABI는 그대로 유지하고, 공통 실행기 어댑터에
// 필요한 PRE-P2P 속도와 실행 정책은 확장 결과에서만 노출한다.
typedef struct VisionControlCommandV3 {
  int command_type;
  int mission;
  int mission_phase;
  int control_phase;
  double vx;
  double vy;
  double wz;
  double pre_p2p_vx;
  double pre_p2p_vy;
  double pre_p2p_wz;
  int action;
  uint64_t action_id;
  int action_category;
  int action_execution_kind;
  double action_yaw_rad;
  int camera_request;
} VisionControlCommandV3;

VisionControlCommandCoordinatorHandle
vision_control_command_coordinator_create(void);
void vision_control_command_coordinator_destroy(
    VisionControlCommandCoordinatorHandle handle);
VisionControlCommand vision_control_command_compute(
    VisionControlCommandCoordinatorHandle handle,
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, double line_vx, double line_vy,
    double line_wz, uint64_t feedback_action_id, int action_acknowledged,
    int action_done);
VisionControlCommand vision_control_command_compute_v2(
    VisionControlCommandCoordinatorHandle handle,
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, int ball_has_ball,
    double line_vx, double line_vy, double line_wz,
    uint64_t feedback_action_id, int action_acknowledged, int action_done);
VisionControlCommandV3 vision_control_command_compute_v3(
    VisionControlCommandCoordinatorHandle handle,
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, int ball_has_ball,
    double line_vx, double line_vy, double line_wz,
    uint64_t feedback_action_id, int action_acknowledged, int action_done);
void vision_control_command_coordinator_reset(
    VisionControlCommandCoordinatorHandle handle);

// ROS의 MissionController::StepPerception과 동일한 공통 파이프라인을
// simulator/Python에서 호출하기 위한 additive C API다. 기존 개별 controller
// 및 control-command API의 ABI는 변경하지 않는다.
typedef void *VisionMissionControllerHandle;

typedef struct VisionPerceptionDetection {
  VisionObjectDetection detection;
  int center_depth_valid;
  double center_depth_m;
  int left_depth_valid;
  double left_depth_m;
  int right_depth_valid;
  double right_depth_m;
} VisionPerceptionDetection;

typedef struct VisionMissionFrameResult {
  VisionControlCommandV3 command;
  int active_mission;
  VisionLineFeatures line_features;
  double line_vx;
  double line_vy;
  double line_wz;
  int line_computed;
  int line_in_recovery;
  int has_ball;
  VisionBallResult ball;
  VisionHurdleResult hurdle;
  VisionGoalResult goal;
  VisionObjectTargets targets;
  VisionGoalPoseObservation goal_pose;
  int imu_rectification_applied;
  int raw_line_count;
  int rectified_line_count;
} VisionMissionFrameResult;

VisionMissionControllerHandle vision_mission_controller_create(void);
void vision_mission_controller_destroy(VisionMissionControllerHandle handle);
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
    uint64_t feedback_action_id, int action_acknowledged, int action_done);
// v1을 보존하면서 READY 전달만 끝에 추가한 ABI다. simulator는 READY를
// 반환해 ROS와 동일하게 다음 긴 라인 action을 선행 예약할 수 있다.
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
    int action_ready);
void vision_mission_controller_reset(VisionMissionControllerHandle handle);

#ifdef __cplusplus
}
#endif
