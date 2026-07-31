#pragma once

#include <cstddef>
#include <optional>
#include <span>

#include "hybrid_localization_core/gaussian_fit_quality.hpp"
#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/gaussian_mixture_update.hpp"

namespace hybrid_localization
{

/// Optional evidence produced by other localization-core stages.
struct LocalizationHealthEvidence
{
  /// Measurement-update diagnostics. Component ordering must match mixture.
  const GaussianMixtureUpdate * measurement_update{nullptr};

  /// Gaussian-fit evidence. When present, one entry is required per component.
  std::span<const GaussianFitQuality> fit_quality{};

  /// Normalized recovery failure severity supplied by a future supervisor.
  std::optional<double> recovery_failure_score{};
};

/// Representation-independent localization health evidence.
///
/// This structure intentionally reports raw metrics rather than classifying the
/// belief as healthy or unhealthy. Thresholds, time windows, and hysteresis are
/// responsibilities of the transition supervisor.
struct LocalizationHealthMetrics
{
  std::size_t component_count{0U};

  double represented_weight{0.0};
  double discarded_weight{0.0};
  double dominant_component_weight{0.0};
  double normalized_mixture_entropy{0.0};
  double effective_component_count{0.0};

  /// Component-weighted covariance summaries conditioned on represented mass.
  double weighted_position_variance{0.0};
  double weighted_yaw_variance{0.0};
  double maximum_position_variance{0.0};
  double maximum_yaw_variance{0.0};

  bool has_measurement_update{false};
  double accepted_component_fraction{0.0};
  double accepted_component_weight_fraction{0.0};
  double weighted_mahalanobis_distance_squared{0.0};
  double maximum_mahalanobis_distance_squared{0.0};
  double normalization_evidence{0.0};

  bool has_fit_quality{false};
  double weighted_fit_mean_mahalanobis_distance{0.0};
  double maximum_fit_mahalanobis_distance{0.0};
  double minimum_angular_resultant_length{1.0};

  bool has_recovery_failure_score{false};
  double recovery_failure_score{0.0};
};

/// Evaluate localization health evidence for a Gaussian-mixture belief.
///
/// Component-weighted values are conditioned on represented mass. Empty,
/// fully discarded mixtures are supported and report zero represented metrics.
/// Optional update and fit evidence must correspond to the supplied mixture.
[[nodiscard]] LocalizationHealthMetrics evaluate_localization_health(
  const GaussianMixture & mixture,
  const LocalizationHealthEvidence & evidence = {});

}  // namespace hybrid_localization