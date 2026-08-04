#include "hybrid_localization_core/localization_evidence_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "hybrid_localization_core/particle_statistics.hpp"

namespace hybrid_localization
{
namespace
{

void require_finite(const double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void require_unit_interval(const double value, const char * name)
{
  require_finite(value, name);
  if (value < 0.0 || value > 1.0) {
    throw std::invalid_argument(std::string(name) + " must lie in [0, 1]");
  }
}

void require_nonnegative(const double value, const char * name)
{
  require_finite(value, name);
  if (value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be nonnegative");
  }
}

void require_positive_count(const std::size_t value, const char * name)
{
  if (value == 0U) {
    throw std::invalid_argument(std::string(name) + " must be greater than zero");
  }
}

void require_at_least(
  const double higher,
  const double lower,
  const char * message)
{
  if (higher < lower) {
    throw std::invalid_argument(message);
  }
}

void require_at_most(
  const double lower,
  const double higher,
  const char * message)
{
  if (lower > higher) {
    throw std::invalid_argument(message);
  }
}

void validate_particle_metrics(const ParticleBeliefMetrics & metrics)
{
  require_positive_count(metrics.particle_count, "particle_count");
  require_unit_interval(metrics.retained_cluster_weight, "retained_cluster_weight");
  require_unit_interval(metrics.noise_weight, "noise_weight");
  require_unit_interval(metrics.dominant_cluster_weight, "dominant_cluster_weight");

  if (metrics.retained_cluster_count > metrics.particle_count) {
    throw std::invalid_argument("retained_cluster_count cannot exceed particle_count");
  }
  if (metrics.retained_cluster_count == 0U && metrics.dominant_cluster_weight != 0.0) {
    throw std::invalid_argument("empty clustering cannot have dominant cluster weight");
  }
  if (metrics.retained_cluster_count > 0U && metrics.dominant_cluster_weight <= 0.0) {
    throw std::invalid_argument("retained clusters require positive dominant weight");
  }
  if (metrics.dominant_cluster_weight > metrics.retained_cluster_weight) {
    throw std::invalid_argument("dominant cluster weight cannot exceed retained weight");
  }
  if (std::abs(metrics.retained_cluster_weight + metrics.noise_weight - 1.0) > 1e-9) {
    throw std::invalid_argument("retained and noise particle weight must sum to one");
  }
}

void validate_health_metrics(const LocalizationHealthMetrics & metrics)
{
  require_unit_interval(metrics.represented_weight, "represented_weight");
  require_unit_interval(metrics.discarded_weight, "discarded_weight");
  require_unit_interval(metrics.dominant_component_weight, "dominant_component_weight");
  require_unit_interval(metrics.normalized_mixture_entropy, "normalized_mixture_entropy");
  require_nonnegative(metrics.effective_component_count, "effective_component_count");
  require_nonnegative(metrics.weighted_position_variance, "weighted_position_variance");
  require_nonnegative(metrics.weighted_yaw_variance, "weighted_yaw_variance");

  if (std::abs(metrics.represented_weight + metrics.discarded_weight - 1.0) > 1e-9) {
    throw std::invalid_argument("represented and discarded GMM weight must sum to one");
  }
  if (metrics.component_count == 0U && metrics.represented_weight > 0.0) {
    throw std::invalid_argument("empty GMM cannot have represented weight");
  }
  if (metrics.component_count > 0U && metrics.effective_component_count < 1.0) {
    throw std::invalid_argument("nonempty GMM must have effective component count at least one");
  }

  if (metrics.has_measurement_update) {
    require_unit_interval(
      metrics.accepted_component_weight_fraction,
      "accepted_component_weight_fraction");
    require_nonnegative(
      metrics.weighted_mahalanobis_distance_squared,
      "weighted_mahalanobis_distance_squared");
  }
  if (metrics.has_fit_quality) {
    require_nonnegative(
      metrics.weighted_fit_mean_mahalanobis_distance,
      "weighted_fit_mean_mahalanobis_distance");
    require_unit_interval(
      metrics.minimum_angular_resultant_length,
      "minimum_angular_resultant_length");
  }
  if (metrics.has_recovery_failure_score) {
    require_unit_interval(metrics.recovery_failure_score, "recovery_failure_score");
  }
}

void validate_comparison(const ParticleGmmComparison & comparison)
{
  require_nonnegative(comparison.position_difference, "position_difference");
  require_nonnegative(comparison.absolute_yaw_difference, "absolute_yaw_difference");
  require_nonnegative(
    comparison.belief_mahalanobis_distance_squared,
    "belief_mahalanobis_distance_squared");
  require_unit_interval(
    comparison.particle_mass_supported_by_gmm,
    "particle_mass_supported_by_gmm");
  require_unit_interval(comparison.represented_gmm_weight, "represented_gmm_weight");
  require_unit_interval(comparison.discarded_gmm_weight, "discarded_gmm_weight");
  if (std::abs(
      comparison.represented_gmm_weight + comparison.discarded_gmm_weight - 1.0) > 1e-9)
  {
    throw std::invalid_argument("comparison GMM mass must sum to one");
  }
}

bool health_is_good(
  const LocalizationHealthMetrics & health,
  const LocalizationEvidencePolicyConfig & config)
{
  bool good =
    health.component_count > 0U &&
    health.represented_weight >= config.good_minimum_represented_weight &&
    health.dominant_component_weight >= config.good_minimum_dominant_weight &&
    health.normalized_mixture_entropy <= config.good_maximum_normalized_entropy &&
    health.weighted_position_variance <= config.good_maximum_weighted_position_variance &&
    health.weighted_yaw_variance <= config.good_maximum_weighted_yaw_variance;

  if (health.has_measurement_update) {
    good = good &&
      health.accepted_component_weight_fraction >=
      config.good_minimum_accepted_component_weight_fraction &&
      health.weighted_mahalanobis_distance_squared <=
      config.good_maximum_weighted_mahalanobis_distance_squared;
  }
  if (health.has_fit_quality) {
    good = good &&
      health.weighted_fit_mean_mahalanobis_distance <=
      config.good_maximum_weighted_fit_mean_mahalanobis_distance &&
      health.minimum_angular_resultant_length >=
      config.good_minimum_angular_resultant_length;
  }
  return good;
}

bool health_is_bad(
  const LocalizationHealthMetrics & health,
  const LocalizationEvidencePolicyConfig & config)
{
  if (health.component_count == 0U) {
    return true;
  }

  bool bad =
    health.represented_weight <= config.bad_maximum_represented_weight ||
    health.dominant_component_weight <= config.bad_maximum_dominant_weight ||
    health.normalized_mixture_entropy >= config.bad_minimum_normalized_entropy ||
    health.weighted_position_variance >= config.bad_minimum_weighted_position_variance ||
    health.weighted_yaw_variance >= config.bad_minimum_weighted_yaw_variance;

  if (health.has_measurement_update) {
    bad = bad ||
      health.accepted_component_weight_fraction <=
      config.bad_maximum_accepted_component_weight_fraction ||
      health.weighted_mahalanobis_distance_squared >=
      config.bad_minimum_weighted_mahalanobis_distance_squared;
  }
  if (health.has_fit_quality) {
    bad = bad ||
      health.weighted_fit_mean_mahalanobis_distance >=
      config.bad_minimum_weighted_fit_mean_mahalanobis_distance ||
      health.minimum_angular_resultant_length <=
      config.bad_maximum_angular_resultant_length;
  }
  return bad;
}

bool tracking_is_ambiguous(
  const LocalizationHealthMetrics & health,
  const LocalizationEvidencePolicyConfig & config,
  const LocalizationState state)
{
  if (health.component_count == 0U) {
    return false;
  }

  const double component_budget_fraction =
    static_cast<double>(health.component_count) /
    static_cast<double>(config.maximum_component_count);

  if (state == LocalizationState::tracking_ambiguous) {
    const bool recovered =
      health.normalized_mixture_entropy <= config.ambiguity_exit_maximum_entropy &&
      health.effective_component_count <=
      config.ambiguity_exit_maximum_effective_component_count &&
      health.dominant_component_weight >=
      config.ambiguity_exit_minimum_dominant_weight &&
      component_budget_fraction < config.ambiguity_component_budget_fraction;
    return !recovered;
  }

  return
    health.normalized_mixture_entropy >= config.ambiguity_entry_minimum_entropy ||
    health.effective_component_count >=
    config.ambiguity_entry_minimum_effective_component_count ||
    health.dominant_component_weight <=
    config.ambiguity_entry_maximum_dominant_weight ||
    component_budget_fraction >= config.ambiguity_component_budget_fraction;
}

}  // namespace

ParticleBeliefMetrics summarize_particle_belief(
  const std::span<const WeightedParticle> particles,
  const ParticleClusteringResult & clustering)
{
  const auto normalized = normalize_weights(particles);
  std::vector<bool> assigned(normalized.size(), false);

  ParticleBeliefMetrics metrics;
  metrics.particle_count = normalized.size();
  metrics.retained_cluster_count = clustering.clusters.size();

  for (const auto & cluster : clustering.clusters) {
    double cluster_weight = 0.0;
    for (const std::size_t index : cluster.particle_indices) {
      if (index >= normalized.size()) {
        throw std::invalid_argument("cluster particle index is out of range");
      }
      if (assigned[index]) {
        throw std::invalid_argument("particle index appears more than once in clustering");
      }
      assigned[index] = true;
      cluster_weight += normalized[index].weight;
    }
    if (std::abs(cluster_weight - cluster.weight) > 1e-9) {
      throw std::invalid_argument("cluster weight does not match indexed particle mass");
    }
    metrics.retained_cluster_weight += cluster_weight;
    metrics.dominant_cluster_weight =
      std::max(metrics.dominant_cluster_weight, cluster_weight);
  }

  for (const std::size_t index : clustering.noise_indices) {
    if (index >= normalized.size()) {
      throw std::invalid_argument("noise particle index is out of range");
    }
    if (assigned[index]) {
      throw std::invalid_argument("particle index appears more than once in clustering");
    }
    assigned[index] = true;
    metrics.noise_weight += normalized[index].weight;
  }

  if (std::find(assigned.begin(), assigned.end(), false) != assigned.end()) {
    throw std::invalid_argument("clustering must partition the complete particle set");
  }

  validate_particle_metrics(metrics);
  return metrics;
}

void validate_localization_evidence_policy_config(
  const LocalizationEvidencePolicyConfig & config)
{
  require_unit_interval(config.particle_minimum_retained_weight, "particle_minimum_retained_weight");
  require_unit_interval(config.particle_minimum_dominant_weight, "particle_minimum_dominant_weight");
  require_unit_interval(config.particle_maximum_noise_weight, "particle_maximum_noise_weight");
  require_positive_count(config.particle_maximum_retained_clusters, "particle_maximum_retained_clusters");

  require_unit_interval(
    config.gmm_minimum_available_represented_weight,
    "gmm_minimum_available_represented_weight");

  require_unit_interval(config.good_minimum_represented_weight, "good_minimum_represented_weight");
  require_unit_interval(config.good_minimum_dominant_weight, "good_minimum_dominant_weight");
  require_unit_interval(config.good_maximum_normalized_entropy, "good_maximum_normalized_entropy");
  require_nonnegative(
    config.good_maximum_weighted_position_variance,
    "good_maximum_weighted_position_variance");
  require_nonnegative(config.good_maximum_weighted_yaw_variance, "good_maximum_weighted_yaw_variance");
  require_unit_interval(
    config.good_minimum_accepted_component_weight_fraction,
    "good_minimum_accepted_component_weight_fraction");
  require_nonnegative(
    config.good_maximum_weighted_mahalanobis_distance_squared,
    "good_maximum_weighted_mahalanobis_distance_squared");
  require_nonnegative(
    config.good_maximum_weighted_fit_mean_mahalanobis_distance,
    "good_maximum_weighted_fit_mean_mahalanobis_distance");
  require_unit_interval(
    config.good_minimum_angular_resultant_length,
    "good_minimum_angular_resultant_length");

  require_unit_interval(config.bad_maximum_represented_weight, "bad_maximum_represented_weight");
  require_unit_interval(config.bad_maximum_dominant_weight, "bad_maximum_dominant_weight");
  require_unit_interval(config.bad_minimum_normalized_entropy, "bad_minimum_normalized_entropy");
  require_nonnegative(
    config.bad_minimum_weighted_position_variance,
    "bad_minimum_weighted_position_variance");
  require_nonnegative(config.bad_minimum_weighted_yaw_variance, "bad_minimum_weighted_yaw_variance");
  require_unit_interval(
    config.bad_maximum_accepted_component_weight_fraction,
    "bad_maximum_accepted_component_weight_fraction");
  require_nonnegative(
    config.bad_minimum_weighted_mahalanobis_distance_squared,
    "bad_minimum_weighted_mahalanobis_distance_squared");
  require_nonnegative(
    config.bad_minimum_weighted_fit_mean_mahalanobis_distance,
    "bad_minimum_weighted_fit_mean_mahalanobis_distance");
  require_unit_interval(
    config.bad_maximum_angular_resultant_length,
    "bad_maximum_angular_resultant_length");

  require_at_least(
    config.good_minimum_represented_weight,
    config.bad_maximum_represented_weight,
    "good represented-weight threshold must exceed bad threshold");
  require_at_least(
    config.good_minimum_dominant_weight,
    config.bad_maximum_dominant_weight,
    "good dominant-weight threshold must exceed bad threshold");
  require_at_most(
    config.good_maximum_normalized_entropy,
    config.bad_minimum_normalized_entropy,
    "good entropy threshold must not exceed bad threshold");
  require_at_most(
    config.good_maximum_weighted_position_variance,
    config.bad_minimum_weighted_position_variance,
    "good position-variance threshold must not exceed bad threshold");
  require_at_most(
    config.good_maximum_weighted_yaw_variance,
    config.bad_minimum_weighted_yaw_variance,
    "good yaw-variance threshold must not exceed bad threshold");
  require_at_least(
    config.good_minimum_accepted_component_weight_fraction,
    config.bad_maximum_accepted_component_weight_fraction,
    "good accepted-weight threshold must exceed bad threshold");
  require_at_most(
    config.good_maximum_weighted_mahalanobis_distance_squared,
    config.bad_minimum_weighted_mahalanobis_distance_squared,
    "good Mahalanobis threshold must not exceed bad threshold");
  require_at_most(
    config.good_maximum_weighted_fit_mean_mahalanobis_distance,
    config.bad_minimum_weighted_fit_mean_mahalanobis_distance,
    "good fit threshold must not exceed bad threshold");
  require_at_least(
    config.good_minimum_angular_resultant_length,
    config.bad_maximum_angular_resultant_length,
    "good angular-resultant threshold must exceed bad threshold");

  require_unit_interval(
    config.ambiguity_entry_minimum_entropy,
    "ambiguity_entry_minimum_entropy");
  require_unit_interval(
    config.ambiguity_exit_maximum_entropy,
    "ambiguity_exit_maximum_entropy");
  require_nonnegative(
    config.ambiguity_entry_minimum_effective_component_count,
    "ambiguity_entry_minimum_effective_component_count");
  require_nonnegative(
    config.ambiguity_exit_maximum_effective_component_count,
    "ambiguity_exit_maximum_effective_component_count");
  require_unit_interval(
    config.ambiguity_entry_maximum_dominant_weight,
    "ambiguity_entry_maximum_dominant_weight");
  require_unit_interval(
    config.ambiguity_exit_minimum_dominant_weight,
    "ambiguity_exit_minimum_dominant_weight");
  require_unit_interval(
    config.ambiguity_component_budget_fraction,
    "ambiguity_component_budget_fraction");
  require_positive_count(config.maximum_component_count, "maximum_component_count");
  require_at_least(
    config.ambiguity_entry_minimum_entropy,
    config.ambiguity_exit_maximum_entropy,
    "ambiguity entropy entry must exceed exit threshold");
  require_at_least(
    config.ambiguity_entry_minimum_effective_component_count,
    config.ambiguity_exit_maximum_effective_component_count,
    "ambiguity component-count entry must exceed exit threshold");
  require_at_least(
    config.ambiguity_exit_minimum_dominant_weight,
    config.ambiguity_entry_maximum_dominant_weight,
    "ambiguity dominant-weight exit must exceed entry threshold");

  require_nonnegative(config.shadow_maximum_position_difference, "shadow_maximum_position_difference");
  require_nonnegative(
    config.shadow_maximum_absolute_yaw_difference,
    "shadow_maximum_absolute_yaw_difference");
  require_nonnegative(
    config.shadow_maximum_mahalanobis_distance_squared,
    "shadow_maximum_mahalanobis_distance_squared");
  require_unit_interval(
    config.shadow_minimum_particle_support_weight,
    "shadow_minimum_particle_support_weight");

  require_unit_interval(
    config.local_recovery_success_maximum_failure_score,
    "local_recovery_success_maximum_failure_score");
  require_unit_interval(
    config.local_recovery_failure_minimum_score,
    "local_recovery_failure_minimum_score");
  require_unit_interval(
    config.global_recovery_success_maximum_failure_score,
    "global_recovery_success_maximum_failure_score");
  require_unit_interval(
    config.global_recovery_failure_minimum_score,
    "global_recovery_failure_minimum_score");
  require_unit_interval(
    config.emergency_minimum_recovery_failure_score,
    "emergency_minimum_recovery_failure_score");
  require_unit_interval(
    config.emergency_maximum_represented_weight,
    "emergency_maximum_represented_weight");
  require_at_most(
    config.local_recovery_success_maximum_failure_score,
    config.local_recovery_failure_minimum_score,
    "local recovery success threshold must not exceed failure threshold");
  require_at_most(
    config.global_recovery_success_maximum_failure_score,
    config.global_recovery_failure_minimum_score,
    "global recovery success threshold must not exceed failure threshold");
  require_at_least(
    config.emergency_minimum_recovery_failure_score,
    config.global_recovery_failure_minimum_score,
    "emergency failure threshold must be at least the global failure threshold");
}

TransitionEvidence build_transition_evidence(
  const LocalizationEvidencePolicyInput & input,
  const LocalizationEvidencePolicyConfig & config)
{
  validate_localization_evidence_policy_config(config);
  require_nonnegative(input.timestamp_seconds, "timestamp_seconds");

  TransitionEvidence evidence;
  evidence.timestamp_seconds = input.timestamp_seconds;

  if (input.particle_metrics.has_value()) {
    const auto & particle = *input.particle_metrics;
    validate_particle_metrics(particle);
    evidence.particle_belief_converged =
      particle.retained_cluster_count > 0U &&
      particle.retained_cluster_count <= config.particle_maximum_retained_clusters &&
      particle.retained_cluster_weight >= config.particle_minimum_retained_weight &&
      particle.dominant_cluster_weight >= config.particle_minimum_dominant_weight &&
      particle.noise_weight <= config.particle_maximum_noise_weight;
  }

  if (input.gmm_health.has_value()) {
    const auto & health = *input.gmm_health;
    validate_health_metrics(health);

    evidence.gmm_available =
      health.component_count > 0U &&
      health.represented_weight >= config.gmm_minimum_available_represented_weight;
    evidence.gmm_health_good = evidence.gmm_available && health_is_good(health, config);
    evidence.gmm_health_bad = !evidence.gmm_available || health_is_bad(health, config);
    evidence.tracking_ambiguous =
      evidence.gmm_available && tracking_is_ambiguous(health, config, input.state);

    if (health.has_recovery_failure_score) {
      evidence.local_recovery_succeeded =
        health.recovery_failure_score <=
        config.local_recovery_success_maximum_failure_score;
      evidence.local_recovery_failed =
        health.recovery_failure_score >=
        config.local_recovery_failure_minimum_score;
      evidence.global_recovery_succeeded =
        health.recovery_failure_score <=
        config.global_recovery_success_maximum_failure_score;
      evidence.global_recovery_failed =
        health.recovery_failure_score >=
        config.global_recovery_failure_minimum_score;
      evidence.emergency_global_recovery =
        health.recovery_failure_score >=
        config.emergency_minimum_recovery_failure_score;
    }

    if (health.component_count == 0U ||
      health.represented_weight <= config.emergency_maximum_represented_weight)
    {
      evidence.emergency_global_recovery = true;
    }
  }

  if (input.shadow_comparison.has_value()) {
    const auto & comparison = *input.shadow_comparison;
    validate_comparison(comparison);
    evidence.shadow_agreement_good =
      comparison.position_difference <= config.shadow_maximum_position_difference &&
      comparison.absolute_yaw_difference <=
      config.shadow_maximum_absolute_yaw_difference &&
      comparison.belief_mahalanobis_distance_squared <=
      config.shadow_maximum_mahalanobis_distance_squared &&
      comparison.particle_mass_supported_by_gmm >=
      config.shadow_minimum_particle_support_weight;
  }

  return evidence;
}

}  // namespace hybrid_localization
