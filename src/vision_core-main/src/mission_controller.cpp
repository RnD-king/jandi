#include "vision_core/mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "vision_core/config_loader.hpp"
#include "vision_core/coordinate_rectifier.hpp"

namespace vision_core {
namespace {
bool IsLongLineLocomotionAction(MissionAction action) {
  return action == MissionAction::kStepForwardFive ||
         action == MissionAction::kStepForwardLeft ||
         action == MissionAction::kStepForwardRight ||
         action == MissionAction::kTurnLeftAndStep ||
         action == MissionAction::kTurnRightAndStep;
}

bool IsInsideP2pDeadband(const MotionCommand &command,
                         const P2pMotionConfig &config) {
  const auto inside = [](double value, double deadband) {
    return value == 0.0 || std::abs(value) < std::max(0.0, deadband);
  };
  return inside(command.vx, config.forward_deadband) &&
         inside(command.vy, config.lateral_deadband) &&
         inside(command.wz, config.yaw_deadband);
}
} // namespace

MissionController::MissionController()
    : MissionController(LoadDefaultAlgorithmConfig()) {}

MissionController::MissionController(const MissionControllerConfig &config)
    : config_(config),
      line_controller_(config.line, config.line_observation_dt),
      line_p2p_controller_(config.line_p2p, config.line),
      ball_controller_(config.ball), hurdle_controller_(config.hurdle),
      goal_controller_(config.goal), command_coordinator_(config.command),
      ball_association_tracker_(config.object_association),
      backboard_association_tracker_(config.object_association),
      hurdle_association_tracker_(config.object_association),
      has_ball_(config.initial_has_ball) {
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
}

MotionCommand MissionController::StepLine(const Features &features,
                                          bool *reference_valid,
                                          Features *updated_features) {
  line_controller_.Observe(features, {});
  Features current = features;
  current.in_recovery = line_controller_.InRecovery() ? 1.0 : 0.0;
  const Command command = line_controller_.Compute(current);
  if (reference_valid != nullptr) {
    *reference_valid = !line_controller_.InRecovery();
  }
  if (updated_features != nullptr) {
    current.in_recovery = line_controller_.InRecovery() ? 1.0 : 0.0;
    *updated_features = current;
  }
  return {command.vx, 0.0, command.wz};
}

void MissionController::BeginLineReacquisition() {
  line_controller_.BeginReacquisition();
}

void MissionController::EnterMission(MissionType mission) {
  active_mission_ = mission;
  line_no_action_hold_active_ = false;
  line_no_action_hold_end_sec_ = 0.0;
  if (mission == MissionType::kBall) {
    hurdle_controller_.Reset();
    goal_controller_.Reset();
    hurdle_result_ = {};
    goal_result_ = {};
  } else if (mission == MissionType::kHurdle) {
    goal_controller_.Reset();
    ball_result_ = {};
    goal_result_ = {};
    has_ball_ = false;
  } else if (mission == MissionType::kGoal) {
    hurdle_controller_.Reset();
    hurdle_result_ = {};
  }
}

void MissionController::FinishMission() {
  active_mission_ = MissionType::kLine;
  line_no_action_hold_active_ = false;
  line_no_action_hold_end_sec_ = 0.0;
  ball_result_ = {};
  hurdle_result_ = {};
  goal_result_ = {};
}

MissionFrameResult MissionController::Step(const MissionFrameInput &input) {
  return StepWithLineImageCenter(input, std::nullopt);
}

MissionFrameResult MissionController::StepWithLineImageCenter(
    const MissionFrameInput &input,
    std::optional<double> line_image_center_u) {
  MissionFrameResult output;
  bool line_reference_valid = false;
  bool allow_new_line_locomotion_action =
      input.allow_new_line_locomotion_action;
  ActionExecutionFeedback action_feedback = input.action_feedback;
  if (input.command_transport_enabled) {
    const bool pending_mission_action =
        last_command_.action_id != 0 &&
        last_command_.action_category == ActionCategory::kMission;
    const bool matching_feedback =
        pending_mission_action && input.delivery_feedback.action_id != 0 &&
        input.delivery_feedback.action_id == last_command_.action_id;
    action_feedback.enabled = true;
    action_feedback.action_done =
        matching_feedback && input.delivery_feedback.done;
    action_feedback.action_active =
        pending_mission_action && !action_feedback.action_done;
  }

  const auto compute_line = [&]() {
    FeatureConfig line_feature_config = config_.line_features;
    if (line_image_center_u && std::isfinite(*line_image_center_u) &&
        *line_image_center_u >= 0.0 &&
        *line_image_center_u < static_cast<double>(input.image_width)) {
      line_feature_config.image_center_u = *line_image_center_u;
    }
    const Features features = ComputeLineFeatures(
        input.line_centers, input.image_width, input.image_height,
        line_controller_.InRecovery(), input.previous_vx, input.previous_wz,
        line_feature_config, &line_feature_state_);
    const MotionCommand velocity_command =
        StepLine(features, &line_reference_valid, &output.line_features);
    output.line_command = velocity_command;

    if (config_.command.locomotion_backend ==
            LocomotionBackend::kP2pAction &&
        active_mission_ == MissionType::kLine) {
      LineGuide decision_guide = input.line_decision_guide_override
                                     ? *input.line_decision_guide_override
                                     : output.line_features.guide;
      bool force_line_hold = false;
      const bool pending_line_locomotion =
          last_command_.action_id != 0 &&
          last_command_.mission == MissionType::kLine &&
          last_command_.action_category == ActionCategory::kLocomotion &&
          last_command_.control_phase !=
              ControlPhase::kWaitingQueuedActionAck &&
          last_command_.control_phase !=
              ControlPhase::kWaitingQueuedActionStart;
      const bool matching_feedback =
          pending_line_locomotion && input.delivery_feedback.action_id != 0 &&
          input.delivery_feedback.action_id == last_command_.action_id;
      const bool long_action =
          pending_line_locomotion &&
          IsLongLineLocomotionAction(last_command_.action);

      if (short_line_collection_active_) {
        line_guide_accumulator_.Add(short_line_collection_action_id_,
                                    input.now_sec,
                                    output.line_features.guide);
        if (input.now_sec + 1e-9 < short_line_collection_end_sec_) {
          force_line_hold = true;
        } else {
          const auto accumulated = line_guide_accumulator_.FinishAll(
              short_line_collection_action_id_);
          short_line_collection_active_ = false;
          short_line_collection_action_id_ = 0;
          short_line_collection_end_sec_ = 0.0;
          if (accumulated) decision_guide = *accumulated;
        }
      } else if (long_action) {
        const bool collection_started =
            ready_line_action_id_ != last_command_.action_id &&
            (last_command_.control_phase == ControlPhase::kWaitingActionDone ||
            (matching_feedback &&
             (input.delivery_feedback.acknowledged ||
              input.delivery_feedback.ready ||
              input.delivery_feedback.done)));
        if (collection_started &&
            !line_guide_accumulator_.ActiveFor(last_command_.action_id)) {
          line_guide_accumulator_.Begin(last_command_.action_id,
                                        input.now_sec);
        }
        if (line_guide_accumulator_.ActiveFor(last_command_.action_id)) {
          line_guide_accumulator_.Add(last_command_.action_id, input.now_sec,
                                      output.line_features.guide);
        }
        if (matching_feedback &&
            (input.delivery_feedback.ready || input.delivery_feedback.done) &&
            ready_line_action_id_ != last_command_.action_id) {
          const auto accumulated =
              line_guide_accumulator_.Finish(last_command_.action_id,
                                             input.now_sec);
          if (accumulated) decision_guide = *accumulated;
          if (input.delivery_feedback.ready) {
            ready_line_action_id_ = last_command_.action_id;
          }
        }
      } else if (input.allow_new_line_locomotion_action &&
                 pending_line_locomotion && matching_feedback &&
                 input.delivery_feedback.done) {
        // 짧은 1걸음/제자리회전은 흔들림이 멎은 뒤의 관측으로 다음 동작을
        // 정한다. DONE 프레임부터 고정 시간 동안 정지하며 전체 표본을 모은다.
        line_guide_accumulator_.Reset();
        short_line_collection_action_id_ = last_command_.action_id;
        short_line_collection_end_sec_ =
            input.now_sec +
            std::max(0.0, config_.line_p2p.short_post_collect_sec);
        short_line_collection_active_ = true;
        line_guide_accumulator_.Begin(short_line_collection_action_id_,
                                      input.now_sec);
        line_guide_accumulator_.Add(short_line_collection_action_id_,
                                    input.now_sec,
                                    output.line_features.guide);
        if (input.now_sec + 1e-9 < short_line_collection_end_sec_) {
          force_line_hold = true;
        } else {
          const auto accumulated = line_guide_accumulator_.FinishAll(
              short_line_collection_action_id_);
          short_line_collection_active_ = false;
          short_line_collection_action_id_ = 0;
          short_line_collection_end_sec_ = 0.0;
          if (accumulated) decision_guide = *accumulated;
        }
      } else if (!pending_line_locomotion) {
        line_guide_accumulator_.Reset();
      }

      const MotionCommand p2p_command =
          line_p2p_controller_.Compute(decision_guide);
      const bool no_action_candidate =
          !decision_guide.valid ||
          IsInsideP2pDeadband(p2p_command, config_.command.p2p);

      // 라인을 잃었거나 판단값이 모두 deadband 안이면 실행기가 ACK/DONE을
      // 반환해야 하는 가짜 정지 action을 만들지 않는다. 정해진 시간 동안
      // 현재 자세를 유지한 뒤 새 관측으로 다시 판단한다.
      if (line_no_action_hold_active_) {
        if (input.now_sec + 1e-9 < line_no_action_hold_end_sec_) {
          force_line_hold = true;
        } else {
          line_no_action_hold_active_ = false;
          line_no_action_hold_end_sec_ = 0.0;
        }
      }
      if (!force_line_hold && !line_no_action_hold_active_ &&
          allow_new_line_locomotion_action && !pending_line_locomotion &&
          !short_line_collection_active_ && no_action_candidate) {
        force_line_hold = true;
        const double hold_sec =
            std::max(0.0, config_.line_p2p.no_action_hold_sec);
        if (hold_sec > 0.0) {
          line_no_action_hold_active_ = true;
          line_no_action_hold_end_sec_ = input.now_sec + hold_sec;
        }
      }

      output.line_command =
          force_line_hold || no_action_candidate ? MotionCommand{}
                                                  : p2p_command;
      if (force_line_hold) allow_new_line_locomotion_action = false;
    }
    output.line_computed = true;
  };

  switch (active_mission_) {
  case MissionType::kLine: {
    compute_line();
    ball_result_ = {};
    hurdle_result_ = {};
    goal_result_ = {};

    if (has_ball_) {
      if (config_.enable_goal) {
        BallResult carrying;
        carrying.has_ball = true;
        carrying.mode = BallMode::kLineFollow;
        goal_controller_.UpdateBallState(carrying);
        goal_result_ = goal_controller_.Compute(
            input.goal_target, input.backboard_target, input.goal_pose,
            input.image_width, input.image_height, input.now_sec,
            line_reference_valid, input.camera_feedback,
            action_feedback);
        if (goal_result_.mode != GoalMode::kLineFollow) {
          EnterMission(MissionType::kGoal);
        }
      }
    } else {
      if (config_.enable_hurdle) {
        hurdle_result_ = hurdle_controller_.Compute(
            input.hurdle_target, input.image_width, input.image_height,
            input.now_sec, output.line_command.vx, line_reference_valid,
            input.camera_feedback, action_feedback);
        if (hurdle_result_.mode != HurdleMode::kLineFollow) {
          EnterMission(MissionType::kHurdle);
          break;
        }
      }

      if (config_.enable_ball) {
        ball_result_ = ball_controller_.Compute(
            input.ball_target, input.image_width, input.image_height,
            input.now_sec, output.line_command.vx, line_reference_valid,
            input.camera_feedback, action_feedback);
        if (ball_result_.mode != BallMode::kLineFollow) {
          EnterMission(MissionType::kBall);
        }
      }
    }
    break;
  }
  case MissionType::kBall: {
    const BallMode previous_mode = ball_result_.mode;
    if (previous_mode == BallMode::kPostPickupLineRecovery) compute_line();
    ball_result_ = ball_controller_.Compute(
        input.ball_target, input.image_width, input.image_height,
        input.now_sec, output.line_command.vx, line_reference_valid,
        input.camera_feedback, action_feedback);
    has_ball_ = ball_result_.has_ball;
    if (previous_mode != BallMode::kPostPickupLineRecovery &&
        ball_result_.mode == BallMode::kPostPickupLineRecovery) {
      BeginLineReacquisition();
    }
    if (ball_result_.mode == BallMode::kLineFollow && !ball_result_.active) {
      if (!output.line_computed) BeginLineReacquisition();
      if (has_ball_ && config_.enable_goal) {
        goal_controller_.UpdateBallState(ball_result_);
      }
      FinishMission();
    }
    break;
  }
  case MissionType::kHurdle: {
    hurdle_result_ = hurdle_controller_.Compute(
        input.hurdle_target, input.image_width, input.image_height,
        input.now_sec, 0.0, false, input.camera_feedback,
        action_feedback);
    if (hurdle_result_.mode == HurdleMode::kLineFollow &&
        !hurdle_result_.active) {
      // 허들 진입 전의 공 후보가 허들을 넘은 직후 곧바로 Ball 미션을
      // 활성화하지 않도록, 허들 미션을 완전히 마친 시점에만 지운다.
      ball_controller_.ClearEntryEvidence();
      BeginLineReacquisition();
      FinishMission();
    }
    break;
  }
  case MissionType::kGoal: {
    const GoalMode previous_mode = goal_result_.mode;
    if (previous_mode == GoalMode::kHeadingRecovery) compute_line();
    goal_result_ = goal_controller_.Compute(
        input.goal_target, input.backboard_target, input.goal_pose,
        input.image_width, input.image_height, input.now_sec,
        line_reference_valid, input.camera_feedback, action_feedback);
    has_ball_ = goal_controller_.HasBall();
    ball_controller_.SetHasBall(has_ball_);
    if (previous_mode != GoalMode::kHeadingRecovery &&
        goal_result_.mode == GoalMode::kHeadingRecovery) {
      BeginLineReacquisition();
    }
    if (goal_result_.mode == GoalMode::kLineFollow && !goal_result_.active) {
      if (!output.line_computed) BeginLineReacquisition();
      FinishMission();
    }
    break;
  }
  default:
    active_mission_ = MissionType::kLine;
    compute_line();
    break;
  }

  output.command = command_coordinator_.Compute(
      ball_result_, hurdle_result_, goal_result_, output.line_command,
      input.delivery_feedback, input.now_sec,
      allow_new_line_locomotion_action);
  if (input.delivery_feedback.done &&
      input.delivery_feedback.action_id == ready_line_action_id_) {
    ready_line_action_id_ = 0;
  }
  if (output.command.mission != MissionType::kLine) {
    line_guide_accumulator_.Reset();
    short_line_collection_active_ = false;
    short_line_collection_action_id_ = 0;
    short_line_collection_end_sec_ = 0.0;
    ready_line_action_id_ = 0;
    line_no_action_hold_active_ = false;
    line_no_action_hold_end_sec_ = 0.0;
  }
  last_command_ = output.command;
  output.active_mission = output.command.mission;
  output.ball = ball_result_;
  output.hurdle = hurdle_result_;
  output.goal = goal_result_;
  output.has_ball = has_ball_;
  output.line_in_recovery = line_controller_.InRecovery();
  output.line_features.in_recovery = output.line_in_recovery ? 1.0 : 0.0;
  return output;
}

PerceptionMissionFrameResult
MissionController::StepPerception(const PerceptionFrameInput &input) {
  PerceptionMissionFrameResult output;
  std::vector<Detection> detections;
  detections.reserve(input.detections.size());
  for (const auto &observation : input.detections) {
    Detection detection = observation.detection;
    if (detection.class_id == config_.object_targets.backboard_class_id) {
      const double min_depth = std::max(
          0.0, std::min(config_.backboard_min_depth_m,
                        config_.backboard_max_depth_m));
      const double max_depth = std::max(
          min_depth, std::max(config_.backboard_min_depth_m,
                              config_.backboard_max_depth_m));
      const bool depth_valid = observation.center_depth_m.has_value() &&
          std::isfinite(*observation.center_depth_m) &&
          *observation.center_depth_m >= min_depth &&
          *observation.center_depth_m <= max_depth;
      if (!depth_valid) {
        // 원본 index를 보존하면서 generic target selector가 이 후보를 고르지
        // 않도록 confidence만 무효화한다.
        detection.confidence = -std::numeric_limits<double>::infinity();
      }
    }
    detections.push_back(detection);
  }

  output.perception.raw_line_centers =
      ExtractLineCenters(detections, config_.line_detection);
  output.perception.line_centers = output.perception.raw_line_centers;

  ObjectTargetSelection selection;
  const auto goal_candidates = ExtractObjectTargetCandidates(
      detections, config_.object_targets.goal_class_id,
      config_.object_targets.goal_confidence, config_.object_targets);
  if (!goal_candidates.empty()) {
    // goal은 tracker 대상이 아니므로 기존처럼 현재 프레임의 confidence가 가장
    // 높은 후보를 고르고, 동률일 때만 bbox 면적을 사용한다.
    const auto goal = std::max_element(
        goal_candidates.begin(), goal_candidates.end(),
        [](const ObjectTargetCandidate &lhs,
           const ObjectTargetCandidate &rhs) {
          if (lhs.target.confidence != rhs.target.confidence) {
            return lhs.target.confidence < rhs.target.confidence;
          }
          return lhs.target.area_px < rhs.target.area_px;
        });
    selection.targets.goal = goal->target;
    selection.indices.goal = goal->detection_index;
  }
  const auto ball_selection = ball_association_tracker_.Update(
      ExtractObjectTargetCandidates(
          detections, config_.object_targets.ball_class_id,
          config_.object_targets.ball_confidence, config_.object_targets),
      input.image_width, input.image_height);
  const auto backboard_selection = backboard_association_tracker_.Update(
      ExtractObjectTargetCandidates(
          detections, config_.object_targets.backboard_class_id,
          config_.object_targets.backboard_confidence,
          config_.object_targets),
      input.image_width, input.image_height);
  const auto hurdle_selection = hurdle_association_tracker_.Update(
      ExtractObjectTargetCandidates(
          detections, config_.object_targets.hurdle_class_id,
          config_.object_targets.hurdle_confidence, config_.object_targets),
      input.image_width, input.image_height);
  // goal class는 association tracker 대상이 아니다. 기존 selector 결과를
  // 유지하고 ball/backboard/hurdle만 클래스별 tracker 결과로 교체한다.
  selection.targets.ball = ball_selection.target;
  selection.indices.ball = ball_selection.detection_index;
  selection.targets.backboard = backboard_selection.target;
  selection.indices.backboard = backboard_selection.detection_index;
  selection.targets.hurdle = hurdle_selection.target;
  selection.indices.hurdle = hurdle_selection.detection_index;
  output.perception.targets = selection.targets;

  const bool rectify = input.enable_imu_rectification && input.imu_valid;
  if (rectify) {
    output.perception.line_centers = RectifyPixelPoints(
        output.perception.raw_line_centers, input.intrinsics, input.roll_rad,
        input.pitch_rad);
    const auto rectify_target = [&](std::optional<ObjectTarget> &target) {
      if (!target) return;
      const auto center = RectifyPixelPoints(
          {target->center_px}, input.intrinsics, input.roll_rad,
          input.pitch_rad);
      if (!center.empty()) {
        target->rectified_center_px = center.front();
        target->center_rectified = true;
      }
    };
    rectify_target(output.perception.targets.ball);
    rectify_target(output.perception.targets.goal);
    rectify_target(output.perception.targets.backboard);
    rectify_target(output.perception.targets.hurdle);
    output.perception.imu_rectification_applied = true;
  }

  if (selection.indices.backboard &&
      *selection.indices.backboard < input.detections.size()) {
    const auto &depth = input.detections[*selection.indices.backboard];
    const auto &backboard = output.perception.targets.backboard;
    output.perception.backboard_center_depth_m = depth.center_depth_m;
    output.perception.backboard_left_depth_m = depth.left_depth_m;
    output.perception.backboard_right_depth_m = depth.right_depth_m;
    const bool goal_fine_phase =
        active_mission_ == MissionType::kGoal &&
        (goal_result_.mode == GoalMode::kRlStopping ||
         goal_result_.mode == GoalMode::kFineAdjust ||
         goal_result_.mode == GoalMode::kShoot);
    const bool near_fine_distance =
        depth.center_depth_m &&
        *depth.center_depth_m <=
            std::max(0.0, config_.goal.fine_adjust_start_z_m);
    if (backboard && depth.center_depth_m && depth.left_depth_m &&
        depth.right_depth_m && (near_fine_distance || goal_fine_phase)) {
      const double center_u =
          std::round(backboard->box_px.x + 0.5 * backboard->box_px.width);
      const double left_u = std::round(backboard->box_px.x);
      const double right_u =
          std::round(backboard->box_px.x + backboard->box_px.width - 1.0);
      output.perception.goal_pose = EstimateGoalPoseFromBackboardDepths(
          center_u, *depth.center_depth_m, left_u, *depth.left_depth_m,
          right_u, *depth.right_depth_m, input.intrinsics,
          backboard->confidence);
    }
  }

  MissionFrameInput prepared;
  prepared.line_centers = output.perception.line_centers;
  std::optional<double> line_image_center_u;
  if (std::isfinite(input.intrinsics.cx) && input.intrinsics.cx >= 0.0 &&
      input.intrinsics.cx < static_cast<double>(input.image_width)) {
    line_image_center_u = input.intrinsics.cx;
  }
  prepared.previous_vx = input.previous_vx;
  prepared.previous_wz = input.previous_wz;
  prepared.ball_target = output.perception.targets.ball;
  prepared.hurdle_target = output.perception.targets.hurdle;
  prepared.goal_target = output.perception.targets.goal;
  prepared.backboard_target = output.perception.targets.backboard;
  prepared.goal_pose = output.perception.goal_pose;
  prepared.image_width = input.image_width;
  prepared.image_height = input.image_height;
  prepared.now_sec = input.now_sec;
  prepared.camera_feedback = input.camera_feedback;
  prepared.command_transport_enabled = input.command_transport_enabled;
  prepared.action_feedback = input.action_feedback;
  prepared.delivery_feedback = input.delivery_feedback;
  prepared.allow_new_line_locomotion_action =
      input.allow_new_line_locomotion_action;
  prepared.line_decision_guide_override =
      input.line_decision_guide_override;
  output.mission = StepWithLineImageCenter(prepared, line_image_center_u);
  return output;
}

bool MissionController::UpdateLineP2pTuning(
    const LineP2pConfig &line_p2p,
    const P2pMotionConfig &normal_p2p) {
  if (last_command_.action_id != 0 || short_line_collection_active_ ||
      line_no_action_hold_active_) {
    return false;
  }
  config_.line_p2p = line_p2p;
  config_.command.p2p = normal_p2p;
  line_p2p_controller_ = LineP2pController(config_.line_p2p, config_.line);
  command_coordinator_.UpdateNormalP2pConfig(config_.command.p2p);
  return true;
}

void MissionController::Reset() {
  active_mission_ = MissionType::kLine;
  has_ball_ = config_.initial_has_ball;
  line_controller_.Reset();
  ball_controller_.Reset();
  hurdle_controller_.Reset();
  goal_controller_.Reset();
  command_coordinator_.Reset();
  ball_association_tracker_.Reset();
  backboard_association_tracker_.Reset();
  hurdle_association_tracker_.Reset();
  line_feature_state_.Reset();
  line_guide_accumulator_.Reset();
  short_line_collection_active_ = false;
  short_line_collection_action_id_ = 0;
  short_line_collection_end_sec_ = 0.0;
  ready_line_action_id_ = 0;
  line_no_action_hold_active_ = false;
  line_no_action_hold_end_sec_ = 0.0;
  ball_result_ = {};
  hurdle_result_ = {};
  goal_result_ = {};
  last_command_ = {};
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
}

} // namespace vision_core
