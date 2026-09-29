// Legacy backup: P2P용 compact line guide를 추가하기 직전의 구현이다.
// 처리 순서: [점선 분기 2단계] coordinate_rectifier 뒤에 호출한다.
// 역할: 보정된 점선 중심점들로부터 오차, 기울기, 가시 점 개수 등 8개 특징을 만든다.
// 값 전달: 점 목록과 이전 명령/복구 상태를 인자로 받고 Features 구조체를 반환한다.
// 다음 단계: 반환값은 line_velocity_controller.cpp의 Observe()/Compute()로 간다.

#include "vision_core/line_feature_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

struct LineFit {
  bool valid{false};
  double a{0.0};
  double b{0.0};
};

LineFit FitLine(const std::vector<Point2> &points, std::size_t begin,
                std::size_t end) {
  if (end <= begin + 1 || end > points.size()) return {};

  double sum_v = 0.0;
  double sum_u = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    sum_v += points[index].v;
    sum_u += points[index].u;
  }
  const double inv_n = 1.0 / static_cast<double>(end - begin);
  const double mean_v = sum_v * inv_n;
  const double mean_u = sum_u * inv_n;

  double var_v = 0.0;
  double cov_vu = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    const double dv = points[index].v - mean_v;
    var_v += dv * dv;
    cov_vu += dv * (points[index].u - mean_u);
  }
  if (std::abs(var_v) <= 1e-9) return {};

  const double a = cov_vu / var_v;
  return {true, a, mean_u - a * mean_v};
}
} // namespace

Features ComputeLineFeatures(const std::vector<Point2> &input, int image_width,
                             int image_height, bool previous_in_recovery,
                             double vx_prev, double wz_prev,
                             const FeatureConfig &cfg,
                             LineFeatureState *state) {
  Features f{};
  f.vx_prev = vx_prev;
  f.wz_prev = wz_prev;
  if (image_width <= 1 || image_height <= 1) {
    f.in_recovery = previous_in_recovery ? 1.0 : 0.0;
    return f;
  }

  std::vector<Point2> points;
  points.reserve(input.size());
  for (const Point2 &point : input) {
    if (std::isfinite(point.u) && std::isfinite(point.v)) {
      points.push_back({Clamp(point.u, 0.0, image_width - 1.0),
                        Clamp(point.v, 0.0, image_height - 1.0)});
    }
  }
  std::sort(points.begin(), points.end(), [](const Point2 &a, const Point2 &b) {
    return (a.v == b.v) ? (a.u < b.u) : (a.v > b.v);
  });
  points.resize(std::min(static_cast<std::size_t>(std::max(1, cfg.max_centers)),
                         points.size()));

  f.n_visible = static_cast<double>(points.size());
  if (points.empty()) {
    f.in_recovery = 1.0;
    return f;
  }

  const double cx = cfg.image_center_u >= 0.0
                        ? cfg.image_center_u
                        : 0.5 * static_cast<double>(image_width);
  const double denom = std::max(cx, 1.0);
  f.u_err_near = (points.front().u - cx) / denom;
  f.u_err_lookahead = f.u_err_near;

  if (points.size() >= 2) {
    const double max_v = points.front().v;
    const double min_v = points.back().v;
    const double v_span = max_v - min_v;
    const LineFit global_fit = FitLine(points, 0, points.size());
    if (global_fit.valid) {
      f.slope = global_fit.a / 120.0;

      LineFit far_fit;
      double raw_curve_score = 0.0;
      double evidence_confidence = 0.0;
      const int min_curve_points = std::max(5, cfg.curve_min_points);
      if (static_cast<int>(points.size()) >= min_curve_points &&
          v_span >= std::max(1.0, cfg.curve_min_v_span_px)) {
        const std::size_t local_count = std::min(
            static_cast<std::size_t>(std::max(3, cfg.curve_local_fit_points)),
            points.size() - 2);
        const LineFit near_fit = FitLine(points, 0, local_count);
        far_fit =
            FitLine(points, points.size() - local_count, points.size());
        if (near_fit.valid && far_fit.valid) {
          const double direction_change =
              std::abs(std::atan(far_fit.a) - std::atan(near_fit.a));
          raw_curve_score = Clamp(
              direction_change /
                  std::max(1e-6, cfg.curve_full_scale_angle_rad),
              0.0, 1.0);
          const double point_confidence = Clamp(
              (static_cast<double>(points.size()) -
               static_cast<double>(min_curve_points) + 1.0) /
                  2.0,
              0.0, 1.0);
          const double span_confidence = Clamp(
              (v_span - cfg.curve_min_v_span_px) /
                  std::max(1.0, cfg.curve_min_v_span_px),
              0.0, 1.0);
          evidence_confidence = point_confidence * span_confidence;
        }
      }

      double curve_score = raw_curve_score * evidence_confidence;
      if (state != nullptr) {
        if (!state->initialized) {
          state->filtered_curve_score = 0.0;
          state->initialized = true;
        }
        if (evidence_confidence > 0.0) {
          const double alpha =
              Clamp(cfg.curve_smoothing_alpha * evidence_confidence, 0.0, 1.0);
          state->filtered_curve_score +=
              alpha * (raw_curve_score - state->filtered_curve_score);
        } else {
          state->filtered_curve_score *=
              Clamp(cfg.curve_missing_decay, 0.0, 1.0);
        }
        state->filtered_curve_score =
            Clamp(state->filtered_curve_score, 0.0, 1.0);
        curve_score = state->filtered_curve_score;
      }

      const double min_scale =
          Clamp(cfg.curve_lookahead_min_scale, 0.0, 1.0);
      const double lookahead_scale =
          1.0 - curve_score * (1.0 - min_scale);
      const double v_lookahead = Clamp(
          points.front().v -
              std::max(0.0, cfg.lookahead_delta_v_px) * lookahead_scale,
          min_v, max_v);
      const double global_u =
          global_fit.a * v_lookahead + global_fit.b;
      double lookahead_u = global_u;
      if (far_fit.valid && evidence_confidence > 0.0) {
        const double far_u = far_fit.a * v_lookahead + far_fit.b;
        const double local_weight =
            Clamp(curve_score * evidence_confidence, 0.0, 1.0);
        lookahead_u =
            (1.0 - local_weight) * global_u + local_weight * far_u;
      }
      f.u_err_lookahead = (lookahead_u - cx) / denom;
    }
  }

  bool recovery = previous_in_recovery;
  if (f.n_visible <= cfg.recover_enter_nvis ||
      std::abs(f.u_err_near) > cfg.recover_enter_u) {
    recovery = true;
  }
  if (f.n_visible >= cfg.recover_exit_nvis &&
      std::abs(f.u_err_near) < cfg.recover_exit_u) {
    recovery = false;
  }
  f.in_recovery = recovery ? 1.0 : 0.0;
  if (f.n_visible >= 2.0) {
    const double alpha = Clamp(recovery ? cfg.lookahead_alpha_recovery
                                        : cfg.lookahead_alpha_normal,
                               0.0, 1.0);
    f.u_err_ctrl = (1.0 - alpha) * f.u_err_near +
                   alpha * f.u_err_lookahead;
  } else {
    f.u_err_ctrl = f.u_err_near;
  }
  return f;
}

} // namespace vision_core
