#pragma once

#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

Features ComputeLineFeatures(const std::vector<Point2> &points, int image_width,
                             int image_height, bool previous_in_recovery,
                             double vx_prev, double wz_prev,
                             const FeatureConfig &config,
                             LineFeatureState *state = nullptr);

} // namespace vision_core
