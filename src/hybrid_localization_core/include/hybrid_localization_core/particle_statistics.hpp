#pragma once

#include <span>
#include <vector>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Return a copy of the particles with weights normalized to sum to one.
///
/// Throws std::invalid_argument when:
/// - the input is empty;
/// - any pose value or weight is non-finite;
/// - any weight is negative;
/// - the total weight is zero.
[[nodiscard]] std::vector<WeightedParticle> normalize_weights(
  std::span<const WeightedParticle> particles);

/// Calculate the weighted mean pose.
///
/// Position uses an ordinary weighted arithmetic mean. Yaw uses a circular
/// mean, so orientations around the -pi/+pi boundary are handled correctly.
[[nodiscard]] Pose2d weighted_mean(
  std::span<const WeightedParticle> particles);

/// Calculate the effective sample size:
///
///   N_eff = 1 / sum(w_i^2)
///
/// Particle weights are normalized internally.
[[nodiscard]] double effective_sample_size(
  std::span<const WeightedParticle> particles);

}  // namespace hybrid_localization