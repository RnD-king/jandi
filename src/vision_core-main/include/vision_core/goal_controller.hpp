#pragma once

#include <deque>
#include <optional>

#include "vision_core/ball_controller.hpp"
#include "vision_core/types.hpp"

namespace vision_core {

enum class GoalMode {
  kLineFollow = 0,
  // 외부 숫자 호환을 위해 보존한다. 현재 post-pickup 대기는 LINE_FOLLOW를
  // 유지한 채 내부 타이머로 처리하므로 새 실행에서는 이 상태에 진입하지 않는다.
  kPostPickupWait = 1,
  kTiltCameraToGoal = 2,
  kSearch = 3,
  kApproach = 4,
  // 실행 순서는 APPROACH/SEARCH -> RL_STOPPING -> FINE_ADJUST다. 기존
  // 외부 연결에서 사용하는 mode 숫자를 유지하기 때문에 값은 9다.
  kRlStopping = 9,
  kFineAdjust = 5,
  kShoot = 6,
  kReturnCameraToLine = 7,
  kHeadingRecovery = 8,
};

enum class GoalActionRequest {
  kNone = 0,
  kFineAdjust = 1,
  kShoot = 2,
  kFineAdjustHold = 3,
};

struct GoalConfig {
  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{};
  // 공 미션 종료와 라인 재획득 뒤, 라인을 계속 추종하며 카메라를 들기 전까지
  // 기다릴 시간이다. 이 구간에는 백보드 검출을 요구하지 않는다.
  double post_pickup_wait_sec{};
  double camera_motion_timeout_sec{};
  // 골대 시야로 카메라를 올리는 동안 라인 방향으로 계속 직진한다.
  double camera_tilt_forward_vx{};
  // 외부 설정 호환을 위해 보존한다. 현재 백보드 search는 제자리회전 없이
  // 정지 관측하므로 사용하지 않는다.
  double search_wz{};
  double target_u_norm{};
  // 골대 앞에서는 항상 저속으로 접근한다.
  double approach_vx{};
  double approach_wz_gain{};
  double approach_wz_max{};
  // 백보드로 접근하다가 깊이가 이 값 이하면 자세 기반 미세정렬로 바꾼다.
  // 시작 거리는 공통 알고리즘 YAML에서 정한다.
  double fine_adjust_start_z_m{};
  // my_cv/ball_and_hoop.py와 같은 투척 위치 계산값이다.
  double hoop_radius_m{};
  double throwing_range_m{};
  // 기존 정면 정렬 설정과의 source compatibility를 위해 보존한다. 현재 슛은
  // 림 중심을 향하도록 실행기에 전달할 yaw를 직접 계산한다.
  double target_yaw_rad{};
  double position_tolerance_m{};
  double yaw_tolerance_rad{};
  double fine_vx_gain{};
  double fine_vy_gain{};
  double fine_wz_gain{};
  // 현재 보행 정책이 거의 반응하지 않는 미소 속도를 피하고, 한 축씩
  // 일정 시간 움직인 뒤 멈춰서 다시 측정한다.
  double fine_translation_min{};
  double fine_wz_min{};
  double fine_vx_max{};
  double fine_vy_max{};
  double fine_wz_max{};
  double fine_pulse_duration_sec{};
  double fine_near_pulse_duration_sec{};
  double fine_near_error_m{};
  double fine_settle_duration_sec{};
  int fine_adjust_window{};
  int fine_adjust_min_hits{};
  // 정렬 완료 뒤 실제 슛 모션을 연결하기 전까지 정지하는 시간이다.
  double shoot_placeholder_sec{};
  double rl_stop_duration_sec{};
};

// RGB-D 입력부가 백보드 bbox의 좌/우 끝 3x3 깊이 중앙값으로 계산해 전달한다.
// x는 카메라 오른쪽(+), z는 카메라 전방(+), yaw는 우측이 더 가까울 때 (+)다.
struct GoalPoseObservation {
  bool valid{false};
  double x_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double confidence{0.0};
};

// 중심 depth는 백보드까지의 전방 거리와 중심 위치에, 좌/우 depth는
// 백보드 평면의 yaw 계산에 사용한다.
GoalPoseObservation EstimateGoalPoseFromBackboardDepths(
    double center_u_px, double center_depth_m, double left_u_px,
    double left_depth_m, double right_u_px, double right_depth_m,
    const Intrinsics &intrinsics, double confidence = 1.0);

struct TrackedGoalPose {
  bool stable{false};
  bool visible{false};
  double x_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double confidence{0.0};
};

GoalPoseObservation EstimateGoalPoseFromEdgeDepths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, const Intrinsics &intrinsics,
    double confidence = 1.0);

struct TrackedGoal {
  bool stable{false};
  bool visible{false};
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double confidence{0.0};
};

struct GoalResult {
  bool active{false};
  CameraRequest camera_request{CameraRequest::kNone};
  GoalActionRequest action_request{GoalActionRequest::kNone};
  GoalMode mode{GoalMode::kLineFollow};
  TrackedGoal tracked;
  TrackedGoalPose pose;
  // 백보드 pose와 규정상 림 오프셋으로 계산한 림 중심 방향이다.
  // 로봇 yaw 기준 좌회전(+), 우회전(-)이며 SHOOT action에만 사용한다.
  double shoot_yaw_rad{0.0};
  MotionCommand command;
};

class GoalController {
public:
  GoalController();
  explicit GoalController(const GoalConfig &config);
  // 호환 API: 공 보유 상태와 골대 진입 준비를 켠다. 실제 골대 미션은 라인을
  // 재획득한 뒤 post_pickup_wait_sec 동안 라인 추종을 유지하고 시작한다.
  void StartAfterPickup(double now_sec);
  void SetHasBall(bool has_ball);
  // 매 프레임 BallResult를 연결할 때 사용한다. 공 미션이 LINE_FOLLOW까지
  // 끝난 뒤에만 골대 진입을 허용한다.
  void UpdateBallState(const BallResult &ball_result);
  bool HasBall() const { return has_ball_; }
  // 단일-target 호환 API에서는 전달된 값을 추적 대상(현재는 backboard)으로 본다.
  GoalResult Compute(const std::optional<ObjectTarget> &tracking_target,
                     int image_width, int image_height, double now_sec,
                     bool line_reference_valid,
                     const CameraFeedback &camera_feedback);
  GoalResult Compute(const std::optional<ObjectTarget> &goal_target,
                     const std::optional<ObjectTarget> &backboard_target,
                     const GoalPoseObservation &goal_pose,
                     int image_width, int image_height, double now_sec,
                     bool line_reference_valid,
                     const CameraFeedback &camera_feedback,
                     const ActionExecutionFeedback &action_feedback);
  GoalResult Compute(const std::optional<ObjectTarget> &goal_target,
                     const std::optional<ObjectTarget> &backboard_target,
                     const GoalPoseObservation &goal_pose,
                     int image_width, int image_height, double now_sec,
                     bool line_reference_valid,
                     const CameraFeedback &camera_feedback);
  static const char *ModeName(GoalMode mode);
  void Reset();

private:
  enum class FineMotionPhase { kReady, kPulse, kSettle };

  static double Clamp(double value, double low, double high);
  void UpdateGoalTracker(const std::optional<ObjectTarget> &target,
                         int image_width, int image_height);
  void UpdatePoseTracker(const std::optional<ObjectTarget> &backboard_target,
                         const GoalPoseObservation &goal_pose);
  MotionCommand ComputeApproachCommand() const;
  MotionCommand ComputeFineAdjustCommand(double now_sec);
  MotionCommand MakeFineAdjustPulse() const;
  bool FineAdjustSettled(double now_sec) const;
  void ResetFineMotion();
  bool PoseReadyForFineAdjust() const;
  bool PoseAligned() const;
  double ComputeShootYawRad() const;
  void ClearTracking();

  GoalConfig config_;
  GoalMode mode_{GoalMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> pose_hit_history_;
  std::deque<bool> fine_enter_history_;
  std::deque<bool> fine_adjust_history_;
  int lost_count_{0};
  int pose_lost_count_{0};
  bool has_smoothed_{false};
  bool has_pose_smoothed_{false};
  TrackedGoal tracked_;
  TrackedGoalPose tracked_pose_;
  double state_enter_sec_{0.0};
  FineMotionPhase fine_motion_phase_{FineMotionPhase::kReady};
  MotionCommand fine_pulse_command_;
  double fine_motion_phase_enter_sec_{0.0};
  double fine_pulse_duration_sec_{0.0};
  // ROS ACTION 경로에서 미세걸음 뒤 RL을 다시 켜지 않고 정지자세 ACTION의
  // ACK/DONE을 기다리는 중인지 나타낸다.
  bool fine_adjust_hold_active_{false};
  double shoot_yaw_rad_{0.0};
  bool has_ball_{false};
  bool ball_consumed_{false};
  bool goal_entry_armed_{false};
  bool post_pickup_line_wait_active_{false};
  double post_pickup_line_wait_start_sec_{0.0};
};

} // namespace vision_core
