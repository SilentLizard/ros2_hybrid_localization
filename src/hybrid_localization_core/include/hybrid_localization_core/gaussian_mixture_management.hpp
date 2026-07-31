#pragma once

#include <cstddef>
#include <limits>

#include "hybrid_localization_core/gaussian_mixture.hpp"

namespace hybrid_localization
{

/// Controls normalization and pruning of a Gaussian mixture.
struct GaussianMixtureManagementConfig
{
  /// Components below this normalized absolute weight are removed.
  double minimum_component_weight{0.0};

  /// Maximum number of retained components after weight sorting.
  std::size_t maximum_component_count{
    std::numeric_limits<std::size_t>::max()};

  /// Numerical tolerance used when checking normalized probability mass.
  double mass_tolerance{1e-12};
};

/// Diagnostic result for Gaussian-mixture normalization and pruning.
struct GaussianMixtureManagementResult
{
  GaussianMixture mixture{};
  std::size_t pruned_component_count{0U};
  double pruned_weight{0.0};
  double normalization_scale{1.0};
};

/// Normalize, sort, and prune a Gaussian mixture.
///
/// Input component weights and discarded_weight are first normalized by their
/// total mass. Components are then sorted by descending normalized weight.
/// Components below minimum_component_weight and components beyond
/// maximum_component_count are removed. Their normalized probability mass is
/// transferred to discarded_weight rather than redistributed over survivors.
///
/// Therefore the returned mixture satisfies:
///
///   sum(component.weight) + discarded_weight == 1
///
/// within mass_tolerance.
[[nodiscard]] GaussianMixtureManagementResult manage_gaussian_mixture(
  const GaussianMixture & mixture,
  const GaussianMixtureManagementConfig & config = {});

}  // namespace hybrid_localization