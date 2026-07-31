#pragma once

#include <cstddef>
#include <span>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

struct GaussianFitQualityConfig
{
  double covariance_regularization{1e-9};
};

struct GaussianFitQuality
{
  double mean_mahalanobis_distance{0.0};
  double maximum_mahalanobis_distance{0.0};
  double angular_resultant_length{0.0};
};

[[nodiscard]] GaussianFitQuality evaluate_gaussian_fit_quality(
  std::span<const WeightedParticle> particles,
  std::span<const std::size_t> indices,
  const GaussianComponent & component,
  const GaussianFitQualityConfig & config = {});

}  // namespace hybrid_localization