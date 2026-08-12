#pragma once

#include <cstddef>
#include <span>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Numerical controls for evaluating how well a Gaussian represents particles.
struct GaussianFitQualityConfig
{
  /// Positive diagonal covariance term used to make Mahalanobis evaluation
  /// robust when a fitted covariance is singular or nearly singular.
  double covariance_regularization{1e-9};
};

/// Diagnostics describing Gaussian fit quality for one particle subset.
struct GaussianFitQuality
{
  /// Conditional-weighted mean squared Mahalanobis distance.
  double mean_mahalanobis_distance{0.0};

  /// Largest squared Mahalanobis distance among the selected particles.
  double maximum_mahalanobis_distance{0.0};

  /// Weighted circular resultant length of particle yaw in [0, 1].
  /// Values near one indicate concentrated heading; values near zero indicate
  /// strong angular dispersion or cancellation.
  double angular_resultant_length{0.0};
};

/// Evaluate Gaussian fit quality for the particles referenced by indices.
///
/// Particle weights are interpreted conditionally over the selected subset, so
/// the returned fit metrics describe cluster shape independently of the
/// cluster's absolute probability mass. The supplied Gaussian component is the
/// reference mean/covariance for Mahalanobis evaluation.
///
/// Throws std::invalid_argument for invalid particle/index data, invalid
/// component state, or invalid configuration.
[[nodiscard]] GaussianFitQuality evaluate_gaussian_fit_quality(
  std::span<const WeightedParticle> particles,
  std::span<const std::size_t> indices,
  const GaussianComponent & component,
  const GaussianFitQualityConfig & config = {});

}  // namespace hybrid_localization
