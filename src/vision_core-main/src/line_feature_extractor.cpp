// 처리 순서: [점선 분기 2단계] coordinate_rectifier 뒤에 호출한다.
// 역할: 보정된 점선 중심점들로부터 기존 연속속도 특징과 P2P용 compact
// LineGuide(offset/heading/curvature/confidence)를 함께 만든다.
// 다음 단계: 기존 Features 값은 line_velocity_controller.cpp로 가고,
// LineGuide는 모션 구간 누적 및 P2P action 선택에 사용한다.

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
  if (end <= begin + 1 || end > points.size())
    return {};

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
  if (std::abs(var_v) <= 1e-9)
    return {};

  const double a = cov_vu / var_v;
  return {true, a, mean_u - a * mean_v};
}

double MeanV(const std::vector<Point2> &points, std::size_t begin,
             std::size_t end) {
  if (end <= begin || end > points.size())
    return 0.0;
  double sum = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    sum += points[index].v;
  }
  return sum / static_cast<double>(end - begin);
}

double FitRmsePx(const std::vector<Point2> &points, std::size_t begin,
                 std::size_t end, const LineFit &fit) {
  if (!fit.valid || end <= begin || end > points.size())
    return 0.0;
  double squared_error = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    const double residual = points[index].u - (fit.a * points[index].v + fit.b);
    squared_error += residual * residual;
  }
  return std::sqrt(squared_error / static_cast<double>(end - begin));
}

double WrapAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

LineGuide ComputeLineGuide(const std::vector<Point2> &points, double cx,
                           double denom, const FeatureConfig &cfg) {
  LineGuide guide;
  if (points.size() < 2)
    return guide;

  const std::size_t local_count = std::min(
      static_cast<std::size_t>(std::max(3, cfg.curve_local_fit_points)),
      points.size());
  const std::size_t far_begin = points.size() - local_count;
  const LineFit near_fit = FitLine(points, 0, local_count);
  const LineFit far_fit = FitLine(points, far_begin, points.size());
  if (!near_fit.valid || !far_fit.valid)
    return guide;

  const double near_reference_v = MeanV(points, 0, local_count);
  const double near_reference_u = near_fit.a * near_reference_v + near_fit.b;
  guide.offset = (near_reference_u - cx) / denom;
  // FitLine의 a=du/dv이고 영상에서 진행 방향은 v 감소 방향이다.
  // 따라서 -atan(a)가 진행 방향 기준 image-right(+) 각도다.
  guide.heading_rad = -std::atan(near_fit.a);
  const double far_heading_rad = -std::atan(far_fit.a);
  guide.curvature_rad = WrapAngle(far_heading_rad - guide.heading_rad);

  const int minimum_points = std::max(3, cfg.curve_min_points);
  const double point_confidence =
      Clamp(static_cast<double>(points.size() - 1) /
                static_cast<double>(std::max(1, minimum_points - 1)),
            0.0, 1.0);
  const double v_span = points.front().v - points.back().v;
  const double span_confidence =
      Clamp(v_span / std::max(1.0, cfg.curve_min_v_span_px), 0.0, 1.0);
  const double near_rmse = FitRmsePx(points, 0, local_count, near_fit);
  const double far_rmse = FitRmsePx(points, far_begin, points.size(), far_fit);
  const double fit_confidence =
      1.0 - Clamp(std::max(near_rmse, far_rmse) /
                      std::max(1.0, cfg.guide_fit_rmse_full_scale_px),
                  0.0, 1.0);
  guide.confidence =
      Clamp(point_confidence * span_confidence * fit_confidence, 0.0, 1.0);
  guide.valid = true;
  return guide;
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
  // 중심점이 principal point로 이동해도 좌/우 정규화 크기가 달라지지 않도록
  // 영상 반폭을 분모로 사용한다.
  const double denom = std::max(0.5 * static_cast<double>(image_width), 1.0);
  f.guide = ComputeLineGuide(points, cx, denom, cfg);
  f.u_err_near = (points.front().u - cx) / denom;
  f.u_err_lookahead = f.u_err_near;

  if (points.size() >= 2) {
    const double max_v = points.front().v;
    const double min_v = points.back().v;
    const double v_span = max_v - min_v;
    const LineFit global_fit = FitLine(points, 0, points.size());
    if (global_fit.valid) {
      // u(v)의 기울기와 진행 방향(v 감소)의 부호는 반대다. u error 및
      // LineGuide와 동일하게 image-right가 양수가 되도록 변환한다.
      f.slope = -global_fit.a / 120.0;

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
        far_fit = FitLine(points, points.size() - local_count, points.size());
        if (near_fit.valid && far_fit.valid) {
          const double direction_change =
              std::abs(std::atan(far_fit.a) - std::atan(near_fit.a));
          raw_curve_score = Clamp(
              direction_change / std::max(1e-6, cfg.curve_full_scale_angle_rad),
              0.0, 1.0);
          const double point_confidence =
              Clamp((static_cast<double>(points.size()) -
                     static_cast<double>(min_curve_points) + 1.0) /
                        2.0,
                    0.0, 1.0);
          const double span_confidence =
              Clamp((v_span - cfg.curve_min_v_span_px) /
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

      const double min_scale = Clamp(cfg.curve_lookahead_min_scale, 0.0, 1.0);
      const double lookahead_scale = 1.0 - curve_score * (1.0 - min_scale);
      const double v_lookahead =
          Clamp(points.front().v -
                    std::max(0.0, cfg.lookahead_delta_v_px) * lookahead_scale,
                min_v, max_v);
      const double global_u = global_fit.a * v_lookahead + global_fit.b;
      double lookahead_u = global_u;
      if (far_fit.valid && evidence_confidence > 0.0) {
        const double far_u = far_fit.a * v_lookahead + far_fit.b;
        const double local_weight =
            Clamp(curve_score * evidence_confidence, 0.0, 1.0);
        lookahead_u = (1.0 - local_weight) * global_u + local_weight * far_u;
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
    f.u_err_ctrl = (1.0 - alpha) * f.u_err_near + alpha * f.u_err_lookahead;
  } else {
    f.u_err_ctrl = f.u_err_near;
  }
  return f;
}

} // namespace vision_core
