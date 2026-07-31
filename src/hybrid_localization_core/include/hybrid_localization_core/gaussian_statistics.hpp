#pragma once

#include <span>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Fit one Gaussian component to a weighted particle population.
///
/// The covariance is the weighted population covariance over [x, y, yaw].
/// Angular residuals are calculated using the shortest wrapped difference.
///
/// Particle weights are normalized internally.
[[nodiscard]] GaussianComponent fit_gaussian(
  std::span<const WeightedParticle> particles);

}  // namespace hybrid_localization