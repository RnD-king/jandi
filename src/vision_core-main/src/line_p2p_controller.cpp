#include "vision_core/line_p2p_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

} // namespace

LineP2pController::LineP2pController(const LineP2pConfig &config,
                                     const RuleConfig &line_config)
    : config_(config), line_config_(line_config) {}

MotionCommand LineP2pController::Compute(const LineGuide &guide) const {
  if (!guide.valid || !std::isfinite(guide.offset) ||
      !std::isfinite(guide.heading_rad) ||
      !std::isfinite(guide.curvature_rad)) {
    return {};
  }

  // LOCAL TUNING OVERRIDE
  // 기본 상태에서는 YAML/ROS override가 들어간 config 값을 사용한다.
  // 빠른 재빌드 실험이 필요할 때만 아래 우변 하나를 숫자 literal로 바꾼다.
  // 예: const double offset_gain = 1.20;
  const double offset_gain = config_.offset_gain;
  const double heading_gain = config_.heading_gain;
  const double curvature_gain = config_.curvature_gain;

  const double offset_term = offset_gain * guide.offset;
  const double heading_term = heading_gain * guide.heading_rad;
  const double curvature_term = curvature_gain * guide.curvature_rad;
  const double steering = offset_term + heading_term + curvature_term;
  const double severity = std::abs(offset_term) + std::abs(heading_term) +
                          std::abs(curvature_term);
  const double base_vx =
      line_config_.v_base * std::max(0.0, line_config_.tracking_speed_scale);

  MotionCommand command;
  command.vx = Clamp(base_vx - severity, 0.0, line_config_.cmd_vx_max);
  command.wz = Clamp(-steering, line_config_.cmd_wz_min,
                     line_config_.cmd_wz_max);
  return command;
}

void LineGuideAccumulator::Begin(std::uint64_t action_id, double now_sec) {
  action_id_ = action_id;
  begin_sec_ = std::isfinite(now_sec) ? now_sec : 0.0;
  samples_.clear();
}

void LineGuideAccumulator::Add(std::uint64_t action_id, double now_sec,
                               const LineGuide &guide) {
  if (!ActiveFor(action_id) || !guide.valid || !std::isfinite(now_sec)) return;
  samples_.push_back({now_sec, guide});
}

std::optional<LineGuide>
LineGuideAccumulator::Finish(std::uint64_t action_id, double now_sec) {
  if (!ActiveFor(action_id) || samples_.empty()) {
    Reset();
    return std::nullopt;
  }

  const double end_sec = std::isfinite(now_sec) ? std::max(begin_sec_, now_sec)
                                                : samples_.back().now_sec;
  const double midpoint_sec = begin_sec_ + 0.5 * (end_sec - begin_sec_);
  const double half_duration = std::max(0.0, end_sec - midpoint_sec);
  double offset_sum = 0.0;
  double heading_sum = 0.0;
  double curvature_sum = 0.0;
  double confidence_sum = 0.0;
  double total_weight = 0.0;

  for (const Sample &sample : samples_) {
    if (sample.now_sec < midpoint_sec) continue;
    const double phase = half_duration > 1e-9
                             ? Clamp((sample.now_sec - midpoint_sec) /
                                         half_duration,
                                     0.0, 1.0)
                             : 1.0;
    const double weight = 1.0 + 2.0 * phase;
    offset_sum += weight * sample.guide.offset;
    heading_sum += weight * sample.guide.heading_rad;
    curvature_sum += weight * sample.guide.curvature_rad;
    // confidence는 현재 판단 gain에는 사용하지 않고 진단값만 집계한다.
    confidence_sum += weight * sample.guide.confidence;
    total_weight += weight;
  }

  LineGuide result;
  if (total_weight > 0.0) {
    result.offset = offset_sum / total_weight;
    result.heading_rad = heading_sum / total_weight;
    result.curvature_rad = curvature_sum / total_weight;
    result.confidence = confidence_sum / total_weight;
    result.valid = true;
  }
  Reset();
  return result.valid ? std::optional<LineGuide>(result) : std::nullopt;
}

std::optional<LineGuide>
LineGuideAccumulator::FinishAll(std::uint64_t action_id) {
  if (!ActiveFor(action_id) || samples_.empty()) {
    Reset();
    return std::nullopt;
  }

  LineGuide result;
  for (const Sample &sample : samples_) {
    result.offset += sample.guide.offset;
    result.heading_rad += sample.guide.heading_rad;
    result.curvature_rad += sample.guide.curvature_rad;
    // confidence는 의도 계산에는 쓰지 않고 진단값으로만 평균한다.
    result.confidence += sample.guide.confidence;
  }
  const double count = static_cast<double>(samples_.size());
  result.offset /= count;
  result.heading_rad /= count;
  result.curvature_rad /= count;
  result.confidence /= count;
  result.valid = true;
  Reset();
  return result;
}

void LineGuideAccumulator::Reset() {
  action_id_ = 0;
  begin_sec_ = 0.0;
  samples_.clear();
}

bool LineGuideAccumulator::ActiveFor(std::uint64_t action_id) const {
  return action_id != 0 && action_id_ == action_id;
}

} // namespace vision_core
