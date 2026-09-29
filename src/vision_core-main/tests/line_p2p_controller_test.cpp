#include "vision_core/line_p2p_controller.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>

namespace {
constexpr double kEps = 1e-12;

vision_core::RuleConfig LineConfig() {
  vision_core::RuleConfig config;
  config.v_base = 0.85;
  config.tracking_speed_scale = 0.80;
  config.cmd_vx_max = 1.20;
  config.cmd_wz_min = -1.90;
  config.cmd_wz_max = 1.90;
  return config;
}

vision_core::LineGuide Guide(double offset, double heading,
                             double curvature, double confidence = 1.0) {
  return {offset, heading, curvature, confidence, true};
}
} // namespace

int main() {
  using namespace vision_core;
  const LineP2pController controller({1.0, 1.0, 1.0}, LineConfig());

  const MotionCommand straight = controller.Compute(Guide(0.0, 0.0, 0.0));
  assert(std::abs(straight.vx - 0.68) < kEps);
  assert(std::abs(straight.wz) < kEps);

  const MotionCommand correction =
      controller.Compute(Guide(0.20, 0.10, -0.05));
  assert(std::abs(correction.vx - 0.33) < kEps);
  assert(std::abs(correction.wz + 0.25) < kEps);

  const MotionCommand sharp = controller.Compute(Guide(0.0, 0.0, 0.80));
  assert(std::abs(sharp.vx) < kEps);
  assert(std::abs(sharp.wz + 0.80) < kEps);

  // confidence는 현재 의도 계산에 사용하지 않는다.
  const MotionCommand low_confidence =
      controller.Compute(Guide(0.20, 0.10, -0.05, 0.0));
  assert(std::abs(low_confidence.vx - correction.vx) < kEps);
  assert(std::abs(low_confidence.wz - correction.wz) < kEps);

  LineGuideAccumulator accumulator;
  accumulator.Begin(7, 0.0);
  accumulator.Add(7, 0.0, Guide(-1.0, -1.0, -1.0));
  accumulator.Add(7, 1.0, Guide(-0.8, -0.8, -0.8));
  accumulator.Add(7, 2.0, Guide(0.20, 0.20, 0.20));
  accumulator.Add(7, 3.0, Guide(0.30, 0.30, 0.30));
  accumulator.Add(7, 4.0, Guide(0.40, 0.40, 0.40));
  const auto accumulated = accumulator.Finish(7, 4.0);
  assert(accumulated.has_value());
  // 실제 action 시간 0~4초의 후반 50%만 사용한다. 2, 3, 4초 표본의
  // 선형 가중치가 각각 1, 2, 3이므로 평균은 2.0 / 6이다.
  const double expected = 2.0 / 6.0;
  assert(std::abs(accumulated->offset - expected) < kEps);
  assert(std::abs(accumulated->heading_rad - expected) < kEps);
  assert(std::abs(accumulated->curvature_rad - expected) < kEps);
  assert(!accumulator.ActiveFor(7));

  accumulator.Begin(8, 5.0);
  accumulator.Add(8, 5.0, Guide(-0.30, -0.20, -0.10));
  accumulator.Add(8, 5.5, Guide(0.30, 0.20, 0.10));
  accumulator.Add(8, 6.0, Guide(0.60, 0.40, 0.20));
  const auto all_samples = accumulator.FinishAll(8);
  assert(all_samples.has_value());
  assert(std::abs(all_samples->offset - 0.20) < kEps);
  assert(std::abs(all_samples->heading_rad - (0.40 / 3.0)) < kEps);
  assert(std::abs(all_samples->curvature_rad - (0.20 / 3.0)) < kEps);
  assert(!accumulator.ActiveFor(8));
  return 0;
}
