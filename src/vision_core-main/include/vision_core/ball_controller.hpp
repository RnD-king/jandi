#pragma once

#include <deque>
#include <optional>

#include "vision_core/types.hpp"

namespace vision_core {

enum class BallMode {
  kLineFollow = 0,
  kApproachBall = 1,
  kTiltCameraDownAndApproach = 2,
  // 실행 순서는 TILT_DOWN -> RL_STOPPING -> FINE_ADJUST다. 기존 외부 연결에서
  // 사용하는 mode 숫자를 바꾸지 않기 위해 RL_STOPPING의 값은 10을 유지한다.
  kRlStoppingForPickup = 10,
  kFineAdjustForPickup = 3,
  kPickupBall = 4,
  kVerifyPickup = 5,
  kStandUpAfterPickup = 6,
  kReturnCameraToLine = 7,
  kBallRecoveryForward = 8,
  kBallRecoveryDown = 9,
  // 집기 시퀀스가 끝난 뒤 RL 속도 제어로 후진하고, line controller가
  // 라인을 다시 잡을 때까지 그 복구 명령을 사용하는 상태다.
  kPostPickupBackAway = 11,
  kPostPickupLineRecovery = 12,
  kVerifyPickupObservation = 13,
};

enum class BallActionRequest {
  kNone = 0,
  kFineAdjustForward = 1,
  kPickup = 2,
  kStandUp = 3,
  kVerifyPickup = 4,
};

struct BallConfig {
  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{};
  double far_u_des_norm{};
  // Temporary compatibility fallback for callers that do not provide the
  // continuously-computed line vx.  New callers use far_speed_scale only.
  double far_vx{};
  double far_vx_min{};
  double far_wz_max{};
  double far_heading_gain{};
  double far_slow_by_turn{};
  double far_dv_max{};
  double far_dw_max{};
  double far_speed_scale{};
  // 아래쪽 노이즈만으로 공 미션이 시작되지 않도록, 먼저 이 경계보다
  // 위에서 stable_window/stable_min_hits만큼 안정적으로 보여야 한다.
  double upper_acquire_v_norm{};
  double tilt_down_v_norm{};
  int tilt_down_window{};
  int tilt_down_min_hits{};
  // Retained for source compatibility. The transition now depends only on the
  // rolling center-v hit history, not bbox height.
  double tilt_down_h_norm{};
  double camera_tilt_duration_sec{};
  double camera_settle_sec{};
  double camera_return_duration_sec{};
  double camera_motion_timeout_sec{};
  int hold_cmd_window{};
  double hold_vx_min{};
  double hold_vx_max{};
  double hold_wz_max{};
  double hold_default_vx{};
  double tilt_walk_speed_scale{};
  double tilt_walk_vx_max{};
  // 실제 미세걸음/집기/확인/일어나기 모션이 연결되기 전의 임시 동작이다.
  // 미세조정은 저속 직진으로, 나머지는 정지 명령과 시간 경과로 대신한다.
  double fine_adjust_placeholder_vx{};
  double fine_adjust_placeholder_duration_sec{};
  double pickup_placeholder_duration_sec{};
  double pickup_verification_placeholder_sec{};
  double stand_up_placeholder_sec{};
  int pickup_max_attempts{};
  // VERIFY_PICKUP 동작이 끝난 뒤 공이 이 프레임 수만큼 연속으로 보이지
  // 않으면 손에 들어온 것으로 판단한다. 보이는 경우에는 stable_window /
  // stable_min_hits 판정을 통과해야 재집기를 시도한다.
  int pickup_success_missing_frames{};
  double post_pickup_back_away_vx{};
  double post_pickup_back_away_sec{};
  double rl_stop_duration_sec{};
  double ball_ignore_duration_sec{};
  double near_target_u_norm{};
  double near_target_v_norm{};
  double near_kx{};
  double near_ky{};
  double near_wz_gain{};
  double near_vx_max{};
  double near_vy_max{};
  double near_wz_max{};
  double near_x_tol{};
  double near_y_tol{};
  bool near_use_lateral{};
  double recovery_timeout_sec{};
  int recovery_reacquire_min_hits{};
  double recovery_center_tolerance_norm{};
  double recovery_forward_vx{};
  double recovery_turn_wz{};
};

struct TrackedBall {
  bool stable{false};
  bool visible{false};
  Point2 center_px;
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double area_norm{0.0};
  double confidence{0.0};
};

struct BallResult {
  bool active{false};
  bool reached_pickup_pose{false};
  bool has_ball{false};
  bool pickup_failed{false};
  int pickup_attempt_count{0};
  CameraRequest camera_request{CameraRequest::kNone};
  BallActionRequest action_request{BallActionRequest::kNone};
  BallMode mode{BallMode::kLineFollow};
  TrackedBall tracked;
  MotionCommand command;
};

class BallController {
public:
  BallController();
  explicit BallController(const BallConfig &config);
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec);
  // Compatibility overload: uses timed camera feedback and the caller's line
  // vx.  New camera-aware adapters should use the CameraFeedback overload.
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx);
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx, bool line_reference_valid);
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx, const CameraFeedback &camera_feedback);
  // line_reference_valid must be true only when line_vx belongs to normal
  // line tracking, not to RECOV/coast/search.  The reference is still updated
  // while ball detections are ignored during cooldown.
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx, bool line_reference_valid,
                     const CameraFeedback &camera_feedback);
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx, bool line_reference_valid,
                     const CameraFeedback &camera_feedback,
                     const ActionExecutionFeedback &action_feedback);
  static const char *ModeName(BallMode mode);
  bool HasBall() const { return has_ball_; }
  void SetHasBall(bool has_ball) { has_ball_ = has_ball; }
  bool PickupFailed() const { return pickup_failed_; }
  int PickupAttemptCount() const { return pickup_attempt_count_; }
  // 다른 진입 후보가 우선되는 동안 이전 Ball 진입 프레임이 남지 않게 한다.
  // 공 보유/실패/cooldown 상태와 마지막 정상 line 기준속도는 보존한다.
  void ClearEntryEvidence();
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  static double LimitRate(double previous, double target, double delta);
  void UpdateTracker(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height);
  MotionCommand ComputeFarCommand(const TrackedBall &ball) const;
  MotionCommand ComputeRecoveryCommand() const;
  MotionCommand ComputeTiltCommand(double now_sec) const;
  MotionCommand ComputeFineAdjustPlaceholderCommand() const;
  void PushRecentCommand(const MotionCommand &command);
  void ClearTrackingState(bool clear_line_reference = true);
  void ResetToLineFollow(bool clear_ignore, bool clear_line_reference = true);
  BallConfig config_;
  BallMode mode_{BallMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> upper_acquire_history_;
  int lost_count_{0};
  bool has_smoothed_{false};
  TrackedBall smoothed_;
  std::deque<MotionCommand> recent_commands_;
  MotionCommand last_command_;
  double state_enter_sec_{0.0};
  std::deque<bool> tilt_trigger_history_;
  int recovery_visible_count_{0};
  double last_seen_u_norm_{0.50};
  // Most recent positive vx explicitly marked as normal line tracking.  It is
  // also refreshed during the ball-ignore cooldown for the next mission.
  double last_tracking_line_vx_{0.0};
  // Captured exactly once on LINE_FOLLOW -> APPROACH_FAR. Background line
  // recovery may continue running, but its later vx must not stop ball motion.
  double latched_far_line_vx_{0.0};
  double latched_tilt_vx_{0.0};
  double ignore_ball_until_sec_{0.0};
  bool has_ball_{false};
  bool pickup_failed_{false};
  int pickup_attempt_count_{0};
  bool post_pickup_return_{false};
};

} // namespace vision_core
