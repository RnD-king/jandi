#pragma once

#include <cstdint>

namespace vision_core {

struct Point2 {
  // OpenCV pixel convention: u is positive to image-right and v is positive
  // downward.
  double u{0.0};
  double v{0.0};
};

struct Intrinsics {
  double fx{600.0};
  double fy{600.0};
  double cx{320.0};
  double cy{240.0};
};

struct FeatureConfig {
  int max_centers{};
  double image_center_u{};
  double lookahead_delta_v_px{};
  // 점이 충분할 때 화면 아래/위의 겹치는 지역 직선 방향 차이로 커브를
  // 추정한다. 커브가 강할수록 lookahead 거리를 설정된 하한까지 줄인다.
  int curve_min_points{};
  double curve_min_v_span_px{};
  int curve_local_fit_points{};
  double curve_full_scale_angle_rad{};
  double curve_smoothing_alpha{};
  double curve_missing_decay{};
  double curve_lookahead_min_scale{};
  // Compact LineGuide의 가까운/먼 local fit 잔차가 이 값 이상이면
  // confidence의 fitting 성분을 0으로 본다.
  double guide_fit_rmse_full_scale_px{};
  // 정상 추종은 가까운 점과 먼 lookahead 점을 설정된 비율로 섞는다.
  double lookahead_alpha_normal{};
  double lookahead_alpha_recovery{};
  double recover_enter_nvis{};
  double recover_exit_nvis{};
  double recover_enter_u{};
  double recover_exit_u{};
};

struct LineFeatureState {
  double filtered_curve_score{0.0};
  bool initialized{false};

  void Reset() {
    filtered_curve_score = 0.0;
    initialized = false;
  }
};

// P2P 모션 판단에 필요한 최소 라인 기하 표현이다. 기존 연속속도용
// Features 필드는 호환성과 검증된 RL 보행 성능을 위해 그대로 유지한다.
struct LineGuide {
  // 모든 signed 값은 image-right가 양수다. MotionCommand의 wz는 로봇
  // 좌회전이 양수이므로 controller에서 부호를 한 번 반전한다.
  // 가까운 점군 fitting의 대표 위치를 영상 중심으로 정규화한 값.
  double offset{0.0};
  // 가까운 점군에서 화면 위(진행 방향)로 향하는 방향각.
  double heading_rad{0.0};
  // 먼 점군 방향각 - 가까운 점군 방향각. 오른쪽 커브가 양수다.
  double curvature_rad{0.0};
  // 점 개수, 세로 분포, 두 local fit 잔차를 합친 0..1 기하 신뢰도.
  double confidence{0.0};
  bool valid{false};
};

struct Features {
  // u error와 slope는 모두 image-right가 양수다.
  double u_err_near{0.0};
  double u_err_lookahead{0.0};
  double u_err_ctrl{0.0};
  double slope{0.0};
  double n_visible{0.0};
  double in_recovery{0.0};
  double vx_prev{0.0};
  double wz_prev{0.0};
  LineGuide guide;
};

struct RuleConfig {
  int line_stable_window{};
  int line_stable_min_hits{};
  double line_reacquire_nvis{};
  double line_reacquire_u{};
  double cmd_vx_min{};
  double cmd_vx_max{};
  double cmd_wz_min{};
  double cmd_wz_max{};
  double v_base{};
  // 정상 라인 추종에서 계산된 vx에 설정된 속도 비율을 적용한다.
  double tracking_speed_scale{};
  double k_u{};
  double k_slope{};
  double k_v_u{};
  double k_v_slope{};
  double dv_max{};
  double dw_max{};
  double recover_vx{};
  double recover_wz{};
  double line_recovery_vx_max{};
  double low_visible_n{};
  double no_visible_n{};
  double low_visible_vx{};
  double no_visible_vx{};
  double low_visible_wz_decay{};
  double no_visible_wz_decay{};

  double recover_coast_s{};
  double recover_lookahead_m{};
  double recover_search_delay_s{};
  double recover_sweep_period_s{};
  double recover_search_wz_min{};
  double recover_search_wz_max{};
  double recover_k_bearing{};
  double recover_k_heading{};
  double recover_k_cross_track{};
  double recover_path_backtrack_m{};
  double recover_path_forward_margin_m{};
  double recover_side_memory_alpha{};
};

struct Pose2 {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Command {
  double vx{0.0};
  double wz{0.0};
};

struct Box2 {
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};
};

struct Detection {
  Box2 box;
  double confidence{0.0};
  int class_id{0};
};

struct ObjectTarget {
  int class_id{0};
  double confidence{0.0};
  Box2 box_px;
  Point2 center_px;
  Point2 rectified_center_px;
  double width_px{0.0};
  double height_px{0.0};
  double area_px{0.0};
  bool center_rectified{false};
};

struct MotionCommand {
  // Robot/ROS convention: vx forward(+), vy left(+), wz left-yaw(+).
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

// 외부 실행기와 공유하는 통합 명령 분류다. 숫자값은 C/ROS 연결층에서도
// 그대로 사용하므로 기존 값의 의미를 변경하지 않는다.
enum class CommandType {
  kNone = 0,
  kVelocity = 1,
  kAction = 2,
  kHold = 3,
};

enum class MissionType {
  kNone = 0,
  kLine = 1,
  kBall = 2,
  kGoal = 3,
  kHurdle = 4,
};

enum class MissionAction {
  kNone = 0,
  // ROS ActionCommand.msg와 C API가 공유하는 단일 action 번호 규약.
  kDefaultPosition = 1,
  kDefaultPoseMode = 2,
  kStepForwardHalf = 3,
  kStepBack = 4,
  kLeftSideStep = 5,
  kRightSideStep = 6,
  kTurnLeft = 7,
  kTurnRight = 8,
  kWalkMode = 9,
  kStepForwardOne = 10,
  kStepForwardLeft = 11,
  kStepForwardRight = 12,
  kStepForwardFive = 13,
  kPickBall = 14,
  kRecatch = 15,
  kHurdle = 16,
  kShoot = 17,
  kTurnLeftAndStep = 18,
  kTurnRightAndStep = 19,
};

// action 숫자는 하나의 ROS 토픽으로 전달하지만, 생성 원인과 DONE 처리 규칙은
// 분리한다. Mission은 controller 상태 전이용 단발 동작이고 Locomotion은
// 연속 속도를 양자화한 반복 가능한 보행 블록이다.
enum class ActionCategory {
  kNone = 0,
  kMission = 1,
  kLocomotion = 2,
};

// 실행기 어댑터가 동일 action_id 생명주기를 유지하면서 실제 실행 방식을
// 선택할 때 사용한다. ROS는 항상 action을 실행하고, MuJoCo는
// kVelocityCompatible에서 pre_p2p_motion을 RL 보행기에 줄 수 있다.
enum class ActionExecutionKind {
  kNone = 0,
  kVelocityCompatible = 1,
  kDiscrete = 2,
  kStationary = 3,
};

enum class ControlPhase {
  kMission = 0,
  kRlStopping = 1,
  kWaitingActionAck = 2,
  kWaitingActionDone = 3,
  kWaitingQueuedActionAck = 4,
  kWaitingQueuedActionStart = 5,
  // ACTION 발행 뒤 설정 시간 안에 ACK를 받지 못해 재실행을 금지한 정지 상태다.
  // 늦은 ACK로 같은 동작이 중복 실행되는 것을 막기 위해 Reset 전까지 유지한다.
  kActionAckTimedOut = 6,
};

enum class CameraMode {
  kForward = 0,
  kDown = 1,
  kTransition = 2,
  kGoal = 3,
};

enum class CameraRequest {
  kNone = 0,
  kDown = 1,
  kForward = 2,
  kGoal = 3,
};

struct CameraFeedback {
  CameraMode actual_mode{CameraMode::kForward};
  bool settled{true};
};

// enabled=false이면 기존 고정시간 placeholder를 사용한다. MissionController는
// 실행기 ACK/DONE과 직전 command를 이용해 이 값을 내부 controller에 전달한다.
struct ActionExecutionFeedback {
  bool enabled{false};
  bool action_done{false};
  bool action_active{false};
};

struct ControlCommand {
  CommandType command_type{CommandType::kVelocity};
  MissionType mission{MissionType::kLine};
  int mission_phase{0};
  ControlPhase control_phase{ControlPhase::kMission};
  // controller가 선택한 원래 연속속도 의도다. P2P backend에서도 action으로
  // 양자화하기 전에 보존되며, 최종 wire velocity와 동시에 반환된다.
  MotionCommand pre_p2p_motion;
  // 기존 최종 velocity 출력이다. P2P action/hold에서는 0이며 velocity
  // backend에서만 실행기가 직접 사용한다.
  MotionCommand velocity;
  MissionAction action{MissionAction::kNone};
  ActionCategory action_category{ActionCategory::kNone};
  ActionExecutionKind action_execution_kind{ActionExecutionKind::kNone};
  std::uint64_t action_id{0};
  // SHOOT은 로봇 yaw 기준 signed 목표각을 사용한다. 라인의
  // TURN_LEFT/RIGHT(±_AND_STEP 포함)는 action이 방향을 구분하므로
  // 양수 회전량만 사용하며, 그 외 action은 0이다.
  double action_yaw_rad{0.0};
  CameraRequest camera_request{CameraRequest::kNone};
};

} // namespace vision_core
