#pragma once

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

#include "hybrid_localization_core/gaussian_fit_quality.hpp"
#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Source-particle and fit-quality evidence for one retained component.
///
/// source_indices must identify the particles from which the component was
/// originally fitted or otherwise evaluated. The evidence array supplied to
/// split_gaussian_mixture_components must correspond to mixture component
/// order.
struct GaussianComponentSplitEvidence
{
  std::span<const std::size_t> source_indices{};
  GaussianFitQuality fit_quality{};
};

struct GaussianMixtureSplittingConfig
{
  /// Split when mean fit residual is strictly greater than this threshold.
  double maximum_mean_mahalanobis_distance{
    std::numeric_limits<double>::infinity()};

  /// Split when maximum fit residual is strictly greater than this threshold.
  double maximum_fit_mahalanobis_distance{
    std::numeric_limits<double>::infinity()};

  /// Split when angular concentration is strictly less than this threshold.
  double minimum_angular_resultant_length{0.0};

  /// A candidate requires at least this many source particles.
  std::size_t minimum_source_samples{4U};

  /// Every generated child requires at least this many source particles.
  std::size_t minimum_child_samples{2U};

  /// Every generated child must receive at least this fraction of the
  /// candidate's source-particle weight.
  double minimum_child_weight_fraction{0.1};

  /// Maximum number of parent components split during one call.
  std::size_t maximum_splits{1U};

  double mass_tolerance{1e-12};
};

struct GaussianMixtureSplittingResult
{
  GaussianMixture mixture{};
  std::size_t split_count{0U};

  /// Original component indices that were split, in input component order.
  std::vector<std::size_t> split_component_indices{};
};

/// Split poor-fit Gaussian components using their source particles.
///
/// Eligible components are partitioned deterministically along their dominant
/// covariance direction. The partition is chosen near the weighted median
/// while respecting minimum child sample and weight constraints. Each child is
/// refitted from its source-particle subset, then child weights are rescaled so
/// their sum exactly equals the parent component's stored probability mass.
/// discarded_weight is unchanged.
[[nodiscard]] GaussianMixtureSplittingResult
split_gaussian_mixture_components(
  std::span<const WeightedParticle> particles,
  const GaussianMixture & mixture,
  std::span<const GaussianComponentSplitEvidence> evidence,
  const GaussianMixtureSplittingConfig & config = {});

}  // namespace hybrid_localization
