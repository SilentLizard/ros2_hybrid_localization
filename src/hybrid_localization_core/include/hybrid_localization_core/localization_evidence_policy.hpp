#pragma once

#include <cstddef>
#include <optional>
#include <span>

#include "hybrid_localization_core/localization_health.hpp"
#include "hybrid_localization_core/particle_clustering.hpp"
#include "hybrid_localization_core/particle_gmm_comparison.hpp"
#include "hybrid_localization_core/transition_supervisor.hpp"

namespace hybrid_localization
{

/// Compact particle-belief summary used by the evidence policy.
struct ParticleBeliefMetrics
{
  std::size_t particle_count{0U};
  std::size_t retained_cluster_count{0U};
  double retained_cluster_weight{0.0};
  double noise_weight{0.0};
  double dominant_cluster_weight{0.0};
};

/// Convert a clustering result into normalized policy metrics.
///
/// Particle weights are normalized internally. Cluster and noise indices must
/// be unique, valid, and form a complete partition of the particle set.
[[nodiscard]] ParticleBeliefMetrics summarize_particle_belief(
  std::span<const WeightedParticle> particles,
  const ParticleClusteringResult & clustering);

/// Numerical thresholds that convert raw localization metrics into the
/// instantaneous boolean evidence consumed by TransitionSupervisor.
///
/// Temporal hysteresis remains in TransitionSupervisor. This policy provides
/// numerical deadbands by using stricter "good" thresholds and looser "bad"
/// thresholds, plus separate ambiguity entry and exit limits.
struct LocalizationEvidencePolicyConfig
{
  // Particle convergence.
  double particle_minimum_retained_weight{0.80};
  double particle_minimum_dominant_weight{0.55};
  double particle_maximum_noise_weight{0.20};
  std::size_t particle_maximum_retained_clusters{3U};

  // GMM availability.
  double gmm_minimum_available_represented_weight{0.05};

  // Healthy-entry thresholds.
  double good_minimum_represented_weight{0.80};
  double good_minimum_dominant_weight{0.45};
  double good_maximum_normalized_entropy{0.75};
  double good_maximum_weighted_position_variance{1.00};
  double good_maximum_weighted_yaw_variance{0.50};
  double good_minimum_accepted_component_weight_fraction{0.50};
  double good_maximum_weighted_mahalanobis_distance_squared{9.0};
  double good_maximum_weighted_fit_mean_mahalanobis_distance{3.0};
  double good_minimum_angular_resultant_length{0.50};

  // Bad-entry thresholds. Values must be looser than the corresponding good
  // limits, leaving a neutral deadband where neither good nor bad is asserted.
  double bad_maximum_represented_weight{0.50};
  double bad_maximum_dominant_weight{0.20};
  double bad_minimum_normalized_entropy{0.95};
  double bad_minimum_weighted_position_variance{4.00};
  double bad_minimum_weighted_yaw_variance{1.50};
  double bad_maximum_accepted_component_weight_fraction{0.20};
  double bad_minimum_weighted_mahalanobis_distance_squared{25.0};
  double bad_minimum_weighted_fit_mean_mahalanobis_distance{6.0};
  double bad_maximum_angular_resultant_length{0.20};

  // Ambiguity hysteresis. Entry is used outside tracking_ambiguous; exit is
  // used while already ambiguous.
  double ambiguity_entry_minimum_entropy{0.75};
  double ambiguity_exit_maximum_entropy{0.55};
  double ambiguity_entry_minimum_effective_component_count{2.50};
  double ambiguity_exit_maximum_effective_component_count{1.75};
  double ambiguity_entry_maximum_dominant_weight{0.45};
  double ambiguity_exit_minimum_dominant_weight{0.60};
  double ambiguity_component_budget_fraction{0.90};
  std::size_t maximum_component_count{8U};

  // Shadow agreement.
  double shadow_maximum_position_difference{0.50};
  double shadow_maximum_absolute_yaw_difference{0.35};
  double shadow_maximum_mahalanobis_distance_squared{9.0};
  double shadow_minimum_particle_support_weight{0.80};

  // Recovery and emergency policy.
  double local_recovery_success_maximum_failure_score{0.25};
  double local_recovery_failure_minimum_score{0.75};
  double global_recovery_success_maximum_failure_score{0.15};
  double global_recovery_failure_minimum_score{0.90};
  double emergency_minimum_recovery_failure_score{0.98};
  double emergency_maximum_represented_weight{0.02};
};

/// Inputs available to one instantaneous policy evaluation.
struct LocalizationEvidencePolicyInput
{
  double timestamp_seconds{0.0};
  LocalizationState state{LocalizationState::particle_global};

  std::optional<ParticleBeliefMetrics> particle_metrics{};
  std::optional<LocalizationHealthMetrics> gmm_health{};
  std::optional<ParticleGmmComparison> shadow_comparison{};
};

/// Validate numerical thresholds and hysteresis ordering.
void validate_localization_evidence_policy_config(
  const LocalizationEvidencePolicyConfig & config);

/// Convert raw localization metrics into thresholded transition evidence.
///
/// This function is deterministic and stateless. TransitionSupervisor remains
/// responsible for consecutive-update counts, dwell time, and cooldown.
[[nodiscard]] TransitionEvidence build_transition_evidence(
  const LocalizationEvidencePolicyInput & input,
  const LocalizationEvidencePolicyConfig & config = {});

}  // namespace hybrid_localization
