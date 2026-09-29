#pragma once

#include <optional>
#include <vector>

#include "vision_core/ball_controller.hpp"
#include "vision_core/control_command.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_detection_extractor.hpp"
#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_p2p_controller.hpp"
#include "vision_core/line_velocity_controller.hpp"
#include "vision_core/object_target_extractor.hpp"
#include "vision_core/object_association_tracker.hpp"

namespace vision_core {

struct MissionControllerConfig {
  FeatureConfig line_features;
  RuleConfig line;
  LineP2pConfig line_p2p;
  BallConfig ball;
  HurdleConfig hurdle;
  GoalConfig goal;
  ControlCommandConfig command;
  LineDetectionConfig line_detection;
  ObjectTargetConfig object_targets;
  ObjectAssociationConfig object_association;
  // YOLO backboard 후보 중심의 aligned depth가 이 범위 안에 있어야 실제
  // backboard 관측으로 인정한다.
  double backboard_min_depth_m{};
  double backboard_max_depth_m{};
  double line_observation_dt{};
  bool enable_ball{};
  bool enable_hurdle{};
  bool enable_goal{};
  // Goal 단독 시험처럼 공 집기 과정을 생략하는 구성에서만 사용한다.
  bool initial_has_ball{};
};

struct MissionFrameInput {
  std::vector<Point2> line_centers;
  double previous_vx{0.0};
  double previous_wz{0.0};
  std::optional<ObjectTarget> ball_target;
  std::optional<ObjectTarget> hurdle_target;
  std::optional<ObjectTarget> goal_target;
  std::optional<ObjectTarget> backboard_target;
  GoalPoseObservation goal_pose;
  int image_width{0};
  int image_height{0};
  double now_sec{0.0};
  CameraFeedback camera_feedback;
  // true이면 controller용 ActionExecutionFeedback을 직전 공통 command와
  // delivery_feedback에서 MissionController가 직접 만든다. 실행기 어댑터는
  // controller 상태를 건드리지 않고 ACK/READY/DONE만 반환하면 된다.
  bool command_transport_enabled{false};
  // command_transport_enabled=false인 기존 직접 호출자를 위한 호환 입력이다.
  ActionExecutionFeedback action_feedback;
  CommandDeliveryFeedback delivery_feedback;
  // false이면 인식/특징/FSM과 기존 action feedback 처리는 계속하지만 새로운
  // LINE locomotion action만 시작하지 않는다.
  bool allow_new_line_locomotion_action{true};
  // 단발 튜닝처럼 여러 프레임에서 집계한 guide로 이번 LINE 판단만 수행한다.
  std::optional<LineGuide> line_decision_guide_override;
};

struct MissionFrameResult {
  // 최종 ROS용 P2P action과 MuJoCo가 선택적으로 사용할 양자화 전
  // pre_p2p_motion이 같은 결과에 함께 들어 있다.
  ControlCommand command;
  MissionType active_mission{MissionType::kLine};
  Features line_features;
  MotionCommand line_command;
  bool line_computed{false};
  bool line_in_recovery{true};
  bool has_ball{false};
  BallResult ball;
  HurdleResult hurdle;
  GoalResult goal;
};

// 센서 adapter가 detection 하나에 부가할 수 있는 센서 의존 표본이다.
// depth 영상 해석은 ROS에 남기고, 거리/자세 계산은 core가 수행한다.
struct PerceptionDetection {
  Detection detection;
  std::optional<double> center_depth_m;
  std::optional<double> left_depth_m;
  std::optional<double> right_depth_m;
};

struct PerceptionFrameInput {
  std::vector<PerceptionDetection> detections;
  Intrinsics intrinsics;
  bool enable_imu_rectification{false};
  bool imu_valid{false};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double previous_vx{0.0};
  double previous_wz{0.0};
  int image_width{0};
  int image_height{0};
  double now_sec{0.0};
  CameraFeedback camera_feedback;
  bool command_transport_enabled{false};
  ActionExecutionFeedback action_feedback;
  CommandDeliveryFeedback delivery_feedback;
  bool allow_new_line_locomotion_action{true};
  std::optional<LineGuide> line_decision_guide_override;
};

struct PreparedPerceptionFrame {
  std::vector<Point2> raw_line_centers;
  std::vector<Point2> line_centers;
  ObjectTargets targets;
  // center depth는 후보 검증과 접근 거리 판정에 항상 사용한다. 좌/우 depth
  // 기반 goal_pose는 미세조정 진입 거리 이후에만 유효해진다.
  GoalPoseObservation goal_pose;
  std::optional<double> backboard_center_depth_m;
  std::optional<double> backboard_left_depth_m;
  std::optional<double> backboard_right_depth_m;
  bool imu_rectification_applied{false};
};

struct PerceptionMissionFrameResult {
  MissionFrameResult mission;
  PreparedPerceptionFrame perception;
};

// 전체 미션의 유일한 진입점이다. LINE에서만 진입 후보를 관찰하고, 물체
// 미션이 잠긴 동안에는 해당 controller 하나만 실행한다.
class MissionController {
public:
  // 설치된 공통 algorithm YAML을 읽는다. 파일이나 필수 키가
  // 없으면 불완전한 0 설정으로 계속하지 않고 예외를 던진다.
  MissionController();
  explicit MissionController(const MissionControllerConfig &config);

  MissionFrameResult Step(const MissionFrameInput &input);
  PerceptionMissionFrameResult StepPerception(
      const PerceptionFrameInput &input);
  MissionType ActiveMission() const { return active_mission_; }
  bool HasBall() const { return has_ball_; }
  // 진행 중 action/짧은 동작 후 관측이 없을 때만 LINE P2P gain과 Normal
  // 양자화 기준을 상태 손실 없이 교체한다.
  bool UpdateLineP2pTuning(const LineP2pConfig &line_p2p,
                           const P2pMotionConfig &normal_p2p);
  void Reset();

private:
  MissionFrameResult StepWithLineImageCenter(
      const MissionFrameInput &input,
      std::optional<double> line_image_center_u);
  MotionCommand StepLine(const Features &features, bool *reference_valid,
                         Features *updated_features);
  void BeginLineReacquisition();
  void EnterMission(MissionType mission);
  void FinishMission();

  MissionControllerConfig config_;
  LineVelocityController line_controller_;
  LineP2pController line_p2p_controller_;
  LineGuideAccumulator line_guide_accumulator_;
  bool short_line_collection_active_{false};
  std::uint64_t short_line_collection_action_id_{0};
  double short_line_collection_end_sec_{0.0};
  std::uint64_t ready_line_action_id_{0};
  bool line_no_action_hold_active_{false};
  double line_no_action_hold_end_sec_{0.0};
  BallController ball_controller_;
  HurdleController hurdle_controller_;
  GoalController goal_controller_;
  ControlCommandCoordinator command_coordinator_;
  // 서로 다른 클래스의 identity가 섞이지 않도록 상태를 분리한다.
  ObjectAssociationTracker ball_association_tracker_;
  ObjectAssociationTracker backboard_association_tracker_;
  ObjectAssociationTracker hurdle_association_tracker_;
  LineFeatureState line_feature_state_;
  MissionType active_mission_{MissionType::kLine};
  bool has_ball_{false};
  BallResult ball_result_;
  HurdleResult hurdle_result_;
  GoalResult goal_result_;
  // delivery feedback을 해당 Mission action에만 라우팅하기 위한 직전 출력이다.
  ControlCommand last_command_;
};

} // namespace vision_core
