#include "hybrid_localization_core/localization_health.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hybrid_localization_core/adaptive_recovery_sampling.hpp"

namespace hybrid_localization
{
namespace
{

constexpr double kMassTolerance = 1e-9;

void validate_finite_nonnegative(const double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(name);
  }
}

void validate_mixture(const GaussianMixture & mixture)
{
  validate_finite_nonnegative(mixture.discarded_weight, "invalid discarded weight");

  double total_mass = mixture.discarded_weight;
  for (const auto & component : mixture.components) {
    validate_finite_nonnegative(component.weight, "invalid component weight");
    if (!std::isfinite(component.mean.x) || !std::isfinite(component.mean.y) ||
      !std::isfinite(component.mean.yaw))
    {
      throw std::invalid_argument("invalid component mean");
    }
    for (const double value : component.covariance) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("invalid component covariance");
      }
    }
    if (component.covariance[0U] < 0.0 || component.covariance[4U] < 0.0 ||
      component.covariance[8U] < 0.0)
    {
      throw std::invalid_argument("negative component variance");
    }
    total_mass += component.weight;
  }

  if (std::abs(total_mass - 1.0) > kMassTolerance) {
    throw std::invalid_argument("mixture mass must sum to one");
  }
}

}  // namespace

LocalizationHealthMetrics evaluate_localization_health(
  const GaussianMixture & mixture,
  const LocalizationHealthEvidence & evidence)
{
  validate_mixture(mixture);

  LocalizationHealthMetrics metrics;
  metrics.component_count = mixture.components.size();
  metrics.discarded_weight = mixture.discarded_weight;
  metrics.represented_weight = 1.0 - mixture.discarded_weight;
  metrics.normalized_mixture_entropy = normalized_mixture_entropy(mixture);

  std::size_t positive_components = 0U;
  for (const auto & component : mixture.components) {
    if (component.weight > 0.0) {
      ++positive_components;
    }
    metrics.dominant_component_weight =
      std::max(metrics.dominant_component_weight, component.weight);

    const double position_variance = component.covariance[0U] + component.covariance[4U];
    const double yaw_variance = component.covariance[8U];
    metrics.maximum_position_variance =
      std::max(metrics.maximum_position_variance, position_variance);
    metrics.maximum_yaw_variance =
      std::max(metrics.maximum_yaw_variance, yaw_variance);

    if (metrics.represented_weight > 0.0) {
      const double conditioned_weight = component.weight / metrics.represented_weight;
      metrics.weighted_position_variance += conditioned_weight * position_variance;
      metrics.weighted_yaw_variance += conditioned_weight * yaw_variance;
    }
  }

  if (positive_components > 0U) {
    metrics.effective_component_count =
      std::exp(metrics.normalized_mixture_entropy * std::log(static_cast<double>(positive_components)));
  }

  if (evidence.measurement_update != nullptr) {
    const auto & update = *evidence.measurement_update;
    if (update.component_updates.size() != mixture.components.size() ||
      update.mixture.components.size() != mixture.components.size())
    {
      throw std::invalid_argument("measurement update size does not match mixture");
    }
    validate_finite_nonnegative(update.normalization_evidence, "invalid normalization evidence");

    metrics.has_measurement_update = true;
    metrics.normalization_evidence = update.normalization_evidence;

    std::size_t accepted_count = 0U;
    double accepted_weight = 0.0;
    for (std::size_t index = 0U; index < update.component_updates.size(); ++index) {
      const auto & component_update = update.component_updates[index];
      validate_finite_nonnegative(
        component_update.mahalanobis_distance_squared,
        "invalid Mahalanobis distance");
      validate_finite_nonnegative(component_update.likelihood, "invalid likelihood");

      metrics.maximum_mahalanobis_distance_squared = std::max(
        metrics.maximum_mahalanobis_distance_squared,
        component_update.mahalanobis_distance_squared);

      if (metrics.represented_weight > 0.0) {
        const double conditioned_weight =
          mixture.components[index].weight / metrics.represented_weight;
        metrics.weighted_mahalanobis_distance_squared +=
          conditioned_weight * component_update.mahalanobis_distance_squared;
      }

      if (component_update.accepted) {
        ++accepted_count;
        accepted_weight += mixture.components[index].weight;
      }
    }

    if (!update.component_updates.empty()) {
      metrics.accepted_component_fraction =
        static_cast<double>(accepted_count) /
        static_cast<double>(update.component_updates.size());
    }
    if (metrics.represented_weight > 0.0) {
      metrics.accepted_component_weight_fraction =
        accepted_weight / metrics.represented_weight;
    }
  }

  if (!evidence.fit_quality.empty()) {
    if (evidence.fit_quality.size() != mixture.components.size()) {
      throw std::invalid_argument("fit-quality size does not match mixture");
    }

    metrics.has_fit_quality = true;
    for (std::size_t index = 0U; index < evidence.fit_quality.size(); ++index) {
      const auto & quality = evidence.fit_quality[index];
      validate_finite_nonnegative(
        quality.mean_mahalanobis_distance,
        "invalid fit mean Mahalanobis distance");
      validate_finite_nonnegative(
        quality.maximum_mahalanobis_distance,
        "invalid fit maximum Mahalanobis distance");
      if (!std::isfinite(quality.angular_resultant_length) ||
        quality.angular_resultant_length < 0.0 || quality.angular_resultant_length > 1.0)
      {
        throw std::invalid_argument("invalid angular resultant length");
      }

      metrics.maximum_fit_mahalanobis_distance = std::max(
        metrics.maximum_fit_mahalanobis_distance,
        quality.maximum_mahalanobis_distance);
      metrics.minimum_angular_resultant_length = std::min(
        metrics.minimum_angular_resultant_length,
        quality.angular_resultant_length);

      if (metrics.represented_weight > 0.0) {
        const double conditioned_weight =
          mixture.components[index].weight / metrics.represented_weight;
        metrics.weighted_fit_mean_mahalanobis_distance +=
          conditioned_weight * quality.mean_mahalanobis_distance;
      }
    }
  }

  if (evidence.recovery_failure_score.has_value()) {
    const double score = *evidence.recovery_failure_score;
    if (!std::isfinite(score) || score < 0.0 || score > 1.0) {
      throw std::invalid_argument("recovery failure score must lie in [0, 1]");
    }
    metrics.has_recovery_failure_score = true;
    metrics.recovery_failure_score = score;
  }

  return metrics;
}

}  // namespace hybrid_localization