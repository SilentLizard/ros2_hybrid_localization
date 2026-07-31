#pragma once

#include <cstddef>
#include <limits>

#include "hybrid_localization_core/gaussian_mixture.hpp"

namespace hybrid_localization
{

/// Controls deterministic pairwise merging of compatible Gaussian components.
struct GaussianMixtureMergingConfig
{
  /// Maximum symmetric squared Mahalanobis distance between component means.
  double maximum_mahalanobis_distance_squared{0.0};

  /// Maximum absolute wrapped yaw difference in radians.
  double maximum_yaw_difference{0.0};

  /// Positive diagonal regularization added to the summed covariance used for distance.
  double covariance_regularization{1e-12};

  /// Numerical covariance validation tolerances.
  double covariance_symmetry_tolerance{1e-12};
  double covariance_psd_tolerance{1e-12};

  /// Numerical tolerance used when checking total probability mass.
  double mass_tolerance{1e-12};
};

/// Diagnostic result for Gaussian-mixture component merging.
struct GaussianMixtureMergingResult
{
  GaussianMixture mixture{};
  std::size_t merge_count{0U};
};

/// Merge compatible Gaussian components while preserving probability mass and
/// the first two moments of each merged pair.
///
/// The algorithm repeatedly selects the compatible pair with the smallest
/// symmetric Mahalanobis distance. Compatibility also requires the wrapped yaw
/// difference to remain below maximum_yaw_difference.
///
/// Merging preserves discarded_weight and satisfies:
///
///   sum(component.weight) + discarded_weight == 1
///
/// within mass_tolerance.
[[nodiscard]] GaussianMixtureMergingResult merge_gaussian_mixture_components(
  const GaussianMixture & mixture,
  const GaussianMixtureMergingConfig & config = {});

}  // namespace hybrid_localization