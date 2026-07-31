#pragma once

#include <cstddef>
#include <span>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Fit one Gaussian to the complete weighted particle set.
///
/// The returned component has weight 1.0 because the complete particle
/// population is represented.
[[nodiscard]] GaussianComponent fit_gaussian(
  std::span<const WeightedParticle> particles);

/// Fit one Gaussian to selected particles.
///
/// Indices refer to positions in particles. The returned component weight is
/// the selected particles' absolute normalized mass within the complete input
/// particle set.
///
/// Throws std::invalid_argument when:
/// - the particle set is invalid;
/// - indices is empty;
/// - an index is out of range;
/// - an index is duplicated.
[[nodiscard]] GaussianComponent fit_gaussian(
  std::span<const WeightedParticle> particles,
  std::span<const std::size_t> indices);

}  // namespace hybrid_localization