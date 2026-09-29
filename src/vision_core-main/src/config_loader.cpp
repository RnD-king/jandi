#include "vision_core/config_loader.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#ifndef VISION_CORE_DEFAULT_ALGORITHM_CONFIG
#define VISION_CORE_DEFAULT_ALGORITHM_CONFIG "vision_algorithm.yaml"
#endif

namespace vision_core {
namespace {

template <typename T>
void Assign(const YAML::Node &node, const char *key, T &value) {
  if (!node || !node.IsMap() || !node[key]) {
    throw std::runtime_error(std::string("missing required vision algorithm key: ") +
                             key);
  }
  try {
    value = node[key].as<T>();
  } catch (const YAML::Exception &error) {
    throw std::runtime_error(std::string("invalid vision algorithm key '") +
                             key + "': " + error.what());
  }
}

void LoadP2p(const YAML::Node &node, P2pMotionConfig &config) {
  Assign(node, "forward_deadband", config.forward_deadband);
  Assign(node, "lateral_deadband", config.lateral_deadband);
  Assign(node, "yaw_deadband", config.yaw_deadband);
  Assign(node, "long_forward_vx", config.long_forward_vx);
  Assign(node, "curve_yaw_threshold", config.curve_yaw_threshold);
  Assign(node, "sharp_turn_yaw_threshold",
         config.sharp_turn_yaw_threshold);
  Assign(node, "turn_in_place_vx_max", config.turn_in_place_vx_max);
  Assign(node, "lateral_dominance_ratio", config.lateral_dominance_ratio);
}

} // namespace

std::string DefaultAlgorithmConfigPath() {
  if (const char *path = std::getenv("VISION_CORE_ALGORITHM_CONFIG");
      path != nullptr && path[0] != '\0') {
    return path;
  }
  return VISION_CORE_DEFAULT_ALGORITHM_CONFIG;
}

MissionControllerConfig LoadAlgorithmConfig(const std::string &path) {
  MissionControllerConfig config{};
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("failed to load vision algorithm config '" + path +
                             "': " + error.what());
  }
  const YAML::Node algorithm = root["vision_algorithm"];
  if (!algorithm || !algorithm.IsMap()) {
    throw std::runtime_error(
        "vision algorithm config must contain a vision_algorithm map: " +
        path);
  }

  const YAML::Node features = algorithm["line_features"];
  Assign(features, "max_centers", config.line_features.max_centers);
  Assign(features, "image_center_u", config.line_features.image_center_u);
  Assign(features, "lookahead_delta_v_px",
         config.line_features.lookahead_delta_v_px);
  Assign(features, "curve_min_points", config.line_features.curve_min_points);
  Assign(features, "curve_min_v_span_px",
         config.line_features.curve_min_v_span_px);
  Assign(features, "curve_local_fit_points",
         config.line_features.curve_local_fit_points);
  Assign(features, "curve_full_scale_angle_rad",
         config.line_features.curve_full_scale_angle_rad);
  Assign(features, "curve_smoothing_alpha",
         config.line_features.curve_smoothing_alpha);
  Assign(features, "curve_missing_decay",
         config.line_features.curve_missing_decay);
  Assign(features, "curve_lookahead_min_scale",
         config.line_features.curve_lookahead_min_scale);
  Assign(features, "guide_fit_rmse_full_scale_px",
         config.line_features.guide_fit_rmse_full_scale_px);
  Assign(features, "lookahead_alpha_normal",
         config.line_features.lookahead_alpha_normal);
  Assign(features, "lookahead_alpha_recovery",
         config.line_features.lookahead_alpha_recovery);
  Assign(features, "recover_enter_nvis",
         config.line_features.recover_enter_nvis);
  Assign(features, "recover_exit_nvis",
         config.line_features.recover_exit_nvis);
  Assign(features, "recover_enter_u", config.line_features.recover_enter_u);
  Assign(features, "recover_exit_u", config.line_features.recover_exit_u);

  const YAML::Node line = algorithm["line"];
  Assign(line, "line_stable_window", config.line.line_stable_window);
  Assign(line, "line_stable_min_hits", config.line.line_stable_min_hits);
  Assign(line, "line_reacquire_nvis", config.line.line_reacquire_nvis);
  Assign(line, "line_reacquire_u", config.line.line_reacquire_u);
  Assign(line, "cmd_vx_min", config.line.cmd_vx_min);
  Assign(line, "cmd_vx_max", config.line.cmd_vx_max);
  Assign(line, "cmd_wz_min", config.line.cmd_wz_min);
  Assign(line, "cmd_wz_max", config.line.cmd_wz_max);
  Assign(line, "v_base", config.line.v_base);
  Assign(line, "tracking_speed_scale", config.line.tracking_speed_scale);
  Assign(line, "k_u", config.line.k_u);
  Assign(line, "k_slope", config.line.k_slope);
  Assign(line, "k_v_u", config.line.k_v_u);
  Assign(line, "k_v_slope", config.line.k_v_slope);
  Assign(line, "dv_max", config.line.dv_max);
  Assign(line, "dw_max", config.line.dw_max);
  Assign(line, "recover_vx", config.line.recover_vx);
  Assign(line, "recover_wz", config.line.recover_wz);
  Assign(line, "line_recovery_vx_max", config.line.line_recovery_vx_max);
  Assign(line, "low_visible_n", config.line.low_visible_n);
  Assign(line, "no_visible_n", config.line.no_visible_n);
  Assign(line, "low_visible_vx", config.line.low_visible_vx);
  Assign(line, "no_visible_vx", config.line.no_visible_vx);
  Assign(line, "low_visible_wz_decay", config.line.low_visible_wz_decay);
  Assign(line, "no_visible_wz_decay", config.line.no_visible_wz_decay);
  Assign(line, "recover_coast_s", config.line.recover_coast_s);
  Assign(line, "recover_lookahead_m", config.line.recover_lookahead_m);
  Assign(line, "recover_search_delay_s", config.line.recover_search_delay_s);
  Assign(line, "recover_sweep_period_s", config.line.recover_sweep_period_s);
  Assign(line, "recover_search_wz_min", config.line.recover_search_wz_min);
  Assign(line, "recover_search_wz_max", config.line.recover_search_wz_max);
  Assign(line, "recover_k_bearing", config.line.recover_k_bearing);
  Assign(line, "recover_k_heading", config.line.recover_k_heading);
  Assign(line, "recover_k_cross_track", config.line.recover_k_cross_track);
  Assign(line, "recover_path_backtrack_m",
         config.line.recover_path_backtrack_m);
  Assign(line, "recover_path_forward_margin_m",
         config.line.recover_path_forward_margin_m);
  Assign(line, "recover_side_memory_alpha",
         config.line.recover_side_memory_alpha);

  const YAML::Node line_p2p = algorithm["line_p2p"];
  Assign(line_p2p, "offset_gain", config.line_p2p.offset_gain);
  Assign(line_p2p, "heading_gain", config.line_p2p.heading_gain);
  Assign(line_p2p, "curvature_gain", config.line_p2p.curvature_gain);
  Assign(line_p2p, "short_post_collect_sec",
         config.line_p2p.short_post_collect_sec);
  Assign(line_p2p, "no_action_hold_sec",
         config.line_p2p.no_action_hold_sec);

  const YAML::Node ball = algorithm["ball"];
  Assign(ball, "stable_window", config.ball.stable_window);
  Assign(ball, "stable_min_hits", config.ball.stable_min_hits);
  Assign(ball, "lost_frames", config.ball.lost_frames);
  Assign(ball, "smooth_alpha", config.ball.smooth_alpha);
  Assign(ball, "far_u_des_norm", config.ball.far_u_des_norm);
  Assign(ball, "far_vx", config.ball.far_vx);
  Assign(ball, "far_vx_min", config.ball.far_vx_min);
  Assign(ball, "far_wz_max", config.ball.far_wz_max);
  Assign(ball, "far_heading_gain", config.ball.far_heading_gain);
  Assign(ball, "far_slow_by_turn", config.ball.far_slow_by_turn);
  Assign(ball, "far_dv_max", config.ball.far_dv_max);
  Assign(ball, "far_dw_max", config.ball.far_dw_max);
  Assign(ball, "far_speed_scale", config.ball.far_speed_scale);
  Assign(ball, "upper_acquire_v_norm", config.ball.upper_acquire_v_norm);
  Assign(ball, "tilt_down_v_norm", config.ball.tilt_down_v_norm);
  Assign(ball, "tilt_down_window", config.ball.tilt_down_window);
  Assign(ball, "tilt_down_min_hits", config.ball.tilt_down_min_hits);
  Assign(ball, "tilt_down_h_norm", config.ball.tilt_down_h_norm);
  Assign(ball, "camera_tilt_duration_sec",
         config.ball.camera_tilt_duration_sec);
  Assign(ball, "camera_settle_sec", config.ball.camera_settle_sec);
  Assign(ball, "camera_return_duration_sec",
         config.ball.camera_return_duration_sec);
  Assign(ball, "camera_motion_timeout_sec",
         config.ball.camera_motion_timeout_sec);
  Assign(ball, "hold_cmd_window", config.ball.hold_cmd_window);
  Assign(ball, "hold_vx_min", config.ball.hold_vx_min);
  Assign(ball, "hold_vx_max", config.ball.hold_vx_max);
  Assign(ball, "hold_wz_max", config.ball.hold_wz_max);
  Assign(ball, "hold_default_vx", config.ball.hold_default_vx);
  Assign(ball, "tilt_walk_speed_scale", config.ball.tilt_walk_speed_scale);
  Assign(ball, "tilt_walk_vx_max", config.ball.tilt_walk_vx_max);
  Assign(ball, "fine_adjust_placeholder_vx",
         config.ball.fine_adjust_placeholder_vx);
  Assign(ball, "fine_adjust_placeholder_duration_sec",
         config.ball.fine_adjust_placeholder_duration_sec);
  Assign(ball, "pickup_placeholder_duration_sec",
         config.ball.pickup_placeholder_duration_sec);
  Assign(ball, "pickup_verification_placeholder_sec",
         config.ball.pickup_verification_placeholder_sec);
  Assign(ball, "stand_up_placeholder_sec",
         config.ball.stand_up_placeholder_sec);
  Assign(ball, "pickup_max_attempts", config.ball.pickup_max_attempts);
  Assign(ball, "pickup_success_missing_frames",
         config.ball.pickup_success_missing_frames);
  Assign(ball, "post_pickup_back_away_vx",
         config.ball.post_pickup_back_away_vx);
  Assign(ball, "post_pickup_back_away_sec",
         config.ball.post_pickup_back_away_sec);
  Assign(ball, "rl_stop_duration_sec", config.ball.rl_stop_duration_sec);
  Assign(ball, "ball_ignore_duration_sec",
         config.ball.ball_ignore_duration_sec);
  Assign(ball, "near_target_u_norm", config.ball.near_target_u_norm);
  Assign(ball, "near_target_v_norm", config.ball.near_target_v_norm);
  Assign(ball, "near_kx", config.ball.near_kx);
  Assign(ball, "near_ky", config.ball.near_ky);
  Assign(ball, "near_wz_gain", config.ball.near_wz_gain);
  Assign(ball, "near_vx_max", config.ball.near_vx_max);
  Assign(ball, "near_vy_max", config.ball.near_vy_max);
  Assign(ball, "near_wz_max", config.ball.near_wz_max);
  Assign(ball, "near_x_tol", config.ball.near_x_tol);
  Assign(ball, "near_y_tol", config.ball.near_y_tol);
  Assign(ball, "near_use_lateral", config.ball.near_use_lateral);
  Assign(ball, "recovery_timeout_sec", config.ball.recovery_timeout_sec);
  Assign(ball, "recovery_reacquire_min_hits",
         config.ball.recovery_reacquire_min_hits);
  Assign(ball, "recovery_center_tolerance_norm",
         config.ball.recovery_center_tolerance_norm);
  Assign(ball, "recovery_forward_vx", config.ball.recovery_forward_vx);
  Assign(ball, "recovery_turn_wz", config.ball.recovery_turn_wz);

  const YAML::Node hurdle = algorithm["hurdle"];
  Assign(hurdle, "stable_window", config.hurdle.stable_window);
  Assign(hurdle, "stable_min_hits", config.hurdle.stable_min_hits);
  Assign(hurdle, "lost_frames", config.hurdle.lost_frames);
  Assign(hurdle, "smooth_alpha", config.hurdle.smooth_alpha);
  Assign(hurdle, "target_u_norm", config.hurdle.target_u_norm);
  Assign(hurdle, "approach_vx", config.hurdle.approach_vx);
  Assign(hurdle, "approach_speed_scale",
         config.hurdle.approach_speed_scale);
  Assign(hurdle, "approach_wz_gain", config.hurdle.approach_wz_gain);
  Assign(hurdle, "approach_wz_max", config.hurdle.approach_wz_max);
  Assign(hurdle, "approach_dw_max", config.hurdle.approach_dw_max);
  Assign(hurdle, "acquire_min_v_norm", config.hurdle.acquire_min_v_norm);
  Assign(hurdle, "tilt_trigger_v_norm",
         config.hurdle.tilt_trigger_v_norm);
  Assign(hurdle, "tilt_trigger_window", config.hurdle.tilt_trigger_window);
  Assign(hurdle, "tilt_trigger_min_hits",
         config.hurdle.tilt_trigger_min_hits);
  Assign(hurdle, "tilt_walk_speed_scale",
         config.hurdle.tilt_walk_speed_scale);
  Assign(hurdle, "tilt_walk_vx_max", config.hurdle.tilt_walk_vx_max);
  Assign(hurdle, "tilt_walk_default_vx",
         config.hurdle.tilt_walk_default_vx);
  Assign(hurdle, "camera_motion_timeout_sec",
         config.hurdle.camera_motion_timeout_sec);
  Assign(hurdle, "contact_walk_placeholder_vx",
         config.hurdle.contact_walk_placeholder_vx);
  Assign(hurdle, "contact_walk_placeholder_sec",
         config.hurdle.contact_walk_placeholder_sec);
  Assign(hurdle, "cross_placeholder_sec",
         config.hurdle.cross_placeholder_sec);
  Assign(hurdle, "hurdle_ignore_duration_sec",
         config.hurdle.hurdle_ignore_duration_sec);
  Assign(hurdle, "rl_stop_duration_sec",
         config.hurdle.rl_stop_duration_sec);
  Assign(hurdle, "recovery_reacquire_min_hits",
         config.hurdle.recovery_reacquire_min_hits);
  Assign(hurdle, "recovery_timeout_sec", config.hurdle.recovery_timeout_sec);
  Assign(hurdle, "recovery_center_tolerance_norm",
         config.hurdle.recovery_center_tolerance_norm);
  Assign(hurdle, "recovery_forward_vx", config.hurdle.recovery_forward_vx);
  Assign(hurdle, "recovery_turn_wz", config.hurdle.recovery_turn_wz);

  const YAML::Node goal = algorithm["goal"];
  Assign(goal, "stable_window", config.goal.stable_window);
  Assign(goal, "stable_min_hits", config.goal.stable_min_hits);
  Assign(goal, "lost_frames", config.goal.lost_frames);
  Assign(goal, "smooth_alpha", config.goal.smooth_alpha);
  Assign(goal, "post_pickup_wait_sec", config.goal.post_pickup_wait_sec);
  Assign(goal, "camera_motion_timeout_sec",
         config.goal.camera_motion_timeout_sec);
  Assign(goal, "camera_tilt_forward_vx", config.goal.camera_tilt_forward_vx);
  Assign(goal, "search_wz", config.goal.search_wz);
  Assign(goal, "target_u_norm", config.goal.target_u_norm);
  Assign(goal, "approach_vx", config.goal.approach_vx);
  Assign(goal, "approach_wz_gain", config.goal.approach_wz_gain);
  Assign(goal, "approach_wz_max", config.goal.approach_wz_max);
  Assign(goal, "fine_adjust_start_z_m", config.goal.fine_adjust_start_z_m);
  Assign(goal, "hoop_radius_m", config.goal.hoop_radius_m);
  Assign(goal, "throwing_range_m", config.goal.throwing_range_m);
  Assign(goal, "target_yaw_rad", config.goal.target_yaw_rad);
  Assign(goal, "position_tolerance_m", config.goal.position_tolerance_m);
  Assign(goal, "yaw_tolerance_rad", config.goal.yaw_tolerance_rad);
  Assign(goal, "fine_vx_gain", config.goal.fine_vx_gain);
  Assign(goal, "fine_vy_gain", config.goal.fine_vy_gain);
  Assign(goal, "fine_wz_gain", config.goal.fine_wz_gain);
  Assign(goal, "fine_translation_min", config.goal.fine_translation_min);
  Assign(goal, "fine_wz_min", config.goal.fine_wz_min);
  Assign(goal, "fine_vx_max", config.goal.fine_vx_max);
  Assign(goal, "fine_vy_max", config.goal.fine_vy_max);
  Assign(goal, "fine_wz_max", config.goal.fine_wz_max);
  Assign(goal, "fine_pulse_duration_sec",
         config.goal.fine_pulse_duration_sec);
  Assign(goal, "fine_near_pulse_duration_sec",
         config.goal.fine_near_pulse_duration_sec);
  Assign(goal, "fine_near_error_m", config.goal.fine_near_error_m);
  Assign(goal, "fine_settle_duration_sec",
         config.goal.fine_settle_duration_sec);
  Assign(goal, "fine_adjust_window", config.goal.fine_adjust_window);
  Assign(goal, "fine_adjust_min_hits", config.goal.fine_adjust_min_hits);
  Assign(goal, "shoot_placeholder_sec", config.goal.shoot_placeholder_sec);
  Assign(goal, "rl_stop_duration_sec", config.goal.rl_stop_duration_sec);

  const YAML::Node line_detection = algorithm["line_detection"];
  Assign(line_detection, "class_id", config.line_detection.class_id);
  Assign(line_detection, "confidence", config.line_detection.confidence);
  Assign(line_detection, "min_box_width", config.line_detection.min_box_width);
  Assign(line_detection, "min_box_height",
         config.line_detection.min_box_height);

  const YAML::Node targets = algorithm["object_targets"];
  Assign(targets, "ball_class_id", config.object_targets.ball_class_id);
  Assign(targets, "goal_class_id", config.object_targets.goal_class_id);
  Assign(targets, "backboard_class_id",
         config.object_targets.backboard_class_id);
  Assign(targets, "hurdle_class_id", config.object_targets.hurdle_class_id);
  Assign(targets, "ball_confidence", config.object_targets.ball_confidence);
  Assign(targets, "goal_confidence", config.object_targets.goal_confidence);
  Assign(targets, "backboard_confidence",
         config.object_targets.backboard_confidence);
  Assign(targets, "hurdle_confidence",
         config.object_targets.hurdle_confidence);
  Assign(targets, "min_box_width", config.object_targets.min_box_width);
  Assign(targets, "min_box_height", config.object_targets.min_box_height);

  const YAML::Node association = algorithm["object_association"];
  Assign(association, "missing_frame_limit",
         config.object_association.missing_frame_limit);
  Assign(association, "max_center_distance_norm",
         config.object_association.max_center_distance_norm);
  Assign(association, "max_size_log_ratio",
         config.object_association.max_size_log_ratio);
  Assign(association, "center_distance_weight",
         config.object_association.center_distance_weight);
  Assign(association, "size_change_weight",
         config.object_association.size_change_weight);

  const YAML::Node command = algorithm["command"];
  Assign(command, "first_action_id", config.command.first_action_id);
  Assign(command, "action_ack_timeout_sec",
         config.command.action_ack_timeout_sec);
  std::string backend;
  Assign(command, "locomotion_backend", backend);
  if (backend == "p2p") {
    config.command.locomotion_backend = LocomotionBackend::kP2pAction;
  } else if (backend == "velocity") {
    config.command.locomotion_backend = LocomotionBackend::kVelocity;
  } else {
    throw std::runtime_error("unknown locomotion_backend in " + path +
                             ": " + backend);
  }
  LoadP2p(command["p2p"], config.command.p2p);
  LoadP2p(command["p2p_fine"], config.command.p2p_fine);
  LoadP2p(command["p2p_recovery"], config.command.p2p_recovery);

  const YAML::Node mission = algorithm["mission"];
  Assign(mission, "backboard_min_depth_m", config.backboard_min_depth_m);
  Assign(mission, "backboard_max_depth_m", config.backboard_max_depth_m);
  Assign(mission, "line_observation_dt", config.line_observation_dt);
  Assign(mission, "enable_ball", config.enable_ball);
  Assign(mission, "enable_hurdle", config.enable_hurdle);
  Assign(mission, "enable_goal", config.enable_goal);
  Assign(mission, "initial_has_ball", config.initial_has_ball);
  return config;
}

MissionControllerConfig LoadDefaultAlgorithmConfig() {
  return LoadAlgorithmConfig(DefaultAlgorithmConfigPath());
}

} // namespace vision_core
