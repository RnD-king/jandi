#pragma once

#include <deque>
#include <optional>

#include "vision_core/ball_controller.hpp"
#include "vision_core/types.hpp"

namespace vision_core {

enum class HurdleMode {
  kLineFollow = 0,
  kApproach = 1,
  kTiltCameraDownAndSlow = 2,
  // 실행 순서는 TILT_DOWN -> RL_STOPPING -> CONTACT_WALK다. 기존 외부
  // 연결에서 사용하는 mode 숫자를 유지하기 때문에 값은 6이다.
  kRlStopping = 6,
  kContactWalk = 3,
  kCross = 4,
  kReturnCameraToLine = 5,
  kRecoveryForward = 7,
  kRecoveryDown = 8,
  kFailed = 9,
};

enum class HurdleActionRequest {
  kNone = 0,
  kContactWalk = 1,
  kCross = 2,
};

struct HurdleConfig {
  int stable_window{};
  int stable_min_hits{};
  int lost_frames{};
  double smooth_alpha{};
  // 임시 허들 접근은 공의 원거리 접근 파라미터와 같은 값을 사용한다.
  double target_u_norm{};
  double approach_vx{};
  double approach_speed_scale{};
  double approach_wz_gain{};
  double approach_wz_max{};
  double approach_dw_max{};
  // 최초 진입은 허들 중심이 원본 화면 높이의 이 비율 이상 내려온 프레임만
  // 안정 검출 hit로 인정한다.
  double acquire_min_v_norm{};
  double tilt_trigger_v_norm{};
  int tilt_trigger_window{};
  int tilt_trigger_min_hits{};
  double tilt_walk_speed_scale{};
  double tilt_walk_vx_max{};
  double tilt_walk_default_vx{};
  double camera_motion_timeout_sec{};
  // 실제 잔발/허들 넘기 모션이 연결되기 전 임시 동작이다.
  double contact_walk_placeholder_vx{};
  double contact_walk_placeholder_sec{};
  double cross_placeholder_sec{};
  double hurdle_ignore_duration_sec{};
  double rl_stop_duration_sec{};
  int recovery_reacquire_min_hits{};
  // 검출 손실 뒤 저속 복구를 유지할 최대 시간. 초과하면 kFailed로 잠긴다.
  double recovery_timeout_sec{};
  double recovery_center_tolerance_norm{};
  double recovery_forward_vx{};
  double recovery_turn_wz{};
};

struct TrackedHurdle {
  bool stable{false};
  bool visible{false};
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double bottom_norm{0.0};
  double confidence{0.0};
};

struct HurdleResult {
  bool active{false};
  CameraRequest camera_request{CameraRequest::kNone};
  HurdleActionRequest action_request{HurdleActionRequest::kNone};
  HurdleMode mode{HurdleMode::kLineFollow};
  TrackedHurdle tracked;
  MotionCommand command;
};

class HurdleController {
public:
  HurdleController();
  explicit HurdleController(const HurdleConfig &config);
  HurdleResult Compute(const std::optional<ObjectTarget> &hurdle_target,
                       int image_width, int image_height, double now_sec,
                       const CameraFeedback &camera_feedback);
  HurdleResult Compute(const std::optional<ObjectTarget> &hurdle_target,
                       int image_width, int image_height, double now_sec,
                       double line_vx, bool line_reference_valid,
                       const CameraFeedback &camera_feedback);
  HurdleResult Compute(const std::optional<ObjectTarget> &hurdle_target,
                       int image_width, int image_height, double now_sec,
                       double line_vx, bool line_reference_valid,
                       const CameraFeedback &camera_feedback,
                       const ActionExecutionFeedback &action_feedback);
  static const char *ModeName(HurdleMode mode);
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  static double LimitRate(double previous, double target, double delta);
  void UpdateTracker(const std::optional<ObjectTarget> &target, int image_width,
                     int image_height);
  MotionCommand ComputeApproachCommand() const;
  MotionCommand ComputeRecoveryCommand() const;
  void ResetToLine(bool clear_ignore);

  HurdleConfig config_;
  HurdleMode mode_{HurdleMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> acquire_history_;
  std::deque<bool> tilt_history_;
  int lost_count_{0};
  int recovery_visible_count_{0};
  bool has_smoothed_{false};
  TrackedHurdle tracked_;
  MotionCommand last_command_;
  double last_tracking_line_vx_{0.0};
  double latched_approach_vx_{0.0};
  double latched_tilt_vx_{0.0};
  double state_enter_sec_{0.0};
  double ignore_until_sec_{0.0};
  double last_seen_u_norm_{0.50};
};

} // namespace vision_core
