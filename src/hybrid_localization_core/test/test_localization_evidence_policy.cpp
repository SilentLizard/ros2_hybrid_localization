#include <gtest/gtest.h>

#include "hybrid_localization_core/localization_evidence_policy.hpp"

#include <array>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hl = hybrid_localization;

namespace
{

hl::LocalizationHealthMetrics healthy_gmm()
{
  hl::LocalizationHealthMetrics metrics;
  metrics.component_count = 1U;
  metrics.represented_weight = 0.95;
  metrics.discarded_weight = 0.05;
  metrics.dominant_component_weight = 0.95;
  metrics.normalized_mixture_entropy = 0.0;
  metrics.effective_component_count = 1.0;
  metrics.weighted_position_variance = 0.2;
  metrics.weighted_yaw_variance = 0.1;
  metrics.has_measurement_update = true;
  metrics.accepted_component_weight_fraction = 1.0;
  metrics.weighted_mahalanobis_distance_squared = 1.0;
  metrics.has_fit_quality = true;
  metrics.weighted_fit_mean_mahalanobis_distance = 1.0;
  metrics.minimum_angular_resultant_length = 0.9;
  return metrics;
}

hl::ParticleGmmComparison agreeing_shadow()
{
  hl::ParticleGmmComparison result;
  result.position_difference = 0.1;
  result.absolute_yaw_difference = 0.1;
  result.belief_mahalanobis_distance_squared = 1.0;
  result.particle_mass_supported_by_gmm = 0.95;
  result.represented_gmm_weight = 0.95;
  result.discarded_gmm_weight = 0.05;
  return result;
}

}  // namespace

TEST(LocalizationEvidencePolicy, SummarizesCompleteParticlePartition)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.4},
    {{0.1, 0.0, 0.0}, 0.4},
    {{5.0, 0.0, 0.0}, 0.2}};

  hl::ParticleClusteringResult clustering;
  clustering.clusters = {{{0U, 1U}, 0.8}};
  clustering.noise_indices = {2U};

  const auto metrics = hl::summarize_particle_belief(particles, clustering);
  EXPECT_EQ(metrics.particle_count, 3U);
  EXPECT_EQ(metrics.retained_cluster_count, 1U);
  EXPECT_NEAR(metrics.retained_cluster_weight, 0.8, 1e-12);
  EXPECT_NEAR(metrics.noise_weight, 0.2, 1e-12);
  EXPECT_NEAR(metrics.dominant_cluster_weight, 0.8, 1e-12);
}

TEST(LocalizationEvidencePolicy, BuildsConvergenceHealthAndShadowEvidence)
{
  hl::LocalizationEvidencePolicyInput input;
  input.timestamp_seconds = 2.0;
  input.state = hl::LocalizationState::mixture_shadow;
  input.particle_metrics = hl::ParticleBeliefMetrics{100U, 1U, 0.9, 0.1, 0.8};
  input.gmm_health = healthy_gmm();
  input.shadow_comparison = agreeing_shadow();

  const auto evidence = hl::build_transition_evidence(input);
  EXPECT_DOUBLE_EQ(evidence.timestamp_seconds, 2.0);
  EXPECT_TRUE(evidence.particle_belief_converged);
  EXPECT_TRUE(evidence.gmm_available);
  EXPECT_TRUE(evidence.gmm_health_good);
  EXPECT_FALSE(evidence.gmm_health_bad);
  EXPECT_FALSE(evidence.tracking_ambiguous);
  EXPECT_TRUE(evidence.shadow_agreement_good);
  EXPECT_FALSE(evidence.emergency_global_recovery);
}

TEST(LocalizationEvidencePolicy, LeavesNeutralDeadbandBetweenGoodAndBad)
{
  auto health = healthy_gmm();
  health.represented_weight = 0.65;
  health.discarded_weight = 0.35;
  health.dominant_component_weight = 0.35;
  health.weighted_position_variance = 2.0;

  hl::LocalizationEvidencePolicyInput input;
  input.gmm_health = health;

  const auto evidence = hl::build_transition_evidence(input);
  EXPECT_TRUE(evidence.gmm_available);
  EXPECT_FALSE(evidence.gmm_health_good);
  EXPECT_FALSE(evidence.gmm_health_bad);
}

TEST(LocalizationEvidencePolicy, UsesStateAwareAmbiguityHysteresis)
{
  auto health = healthy_gmm();
  health.component_count = 3U;
  health.represented_weight = 0.9;
  health.discarded_weight = 0.1;
  health.dominant_component_weight = 0.5;
  health.normalized_mixture_entropy = 0.65;
  health.effective_component_count = 2.0;

  hl::LocalizationEvidencePolicyInput tracking;
  tracking.state = hl::LocalizationState::mixture_tracking;
  tracking.gmm_health = health;
  EXPECT_FALSE(hl::build_transition_evidence(tracking).tracking_ambiguous);

  hl::LocalizationEvidencePolicyInput ambiguous = tracking;
  ambiguous.state = hl::LocalizationState::tracking_ambiguous;
  EXPECT_TRUE(hl::build_transition_evidence(ambiguous).tracking_ambiguous);

  health.normalized_mixture_entropy = 0.5;
  health.effective_component_count = 1.5;
  health.dominant_component_weight = 0.65;
  ambiguous.gmm_health = health;
  EXPECT_FALSE(hl::build_transition_evidence(ambiguous).tracking_ambiguous);
}

TEST(LocalizationEvidencePolicy, MapsRecoveryFailureScore)
{
  auto health = healthy_gmm();
  health.has_recovery_failure_score = true;
  health.recovery_failure_score = 0.8;

  hl::LocalizationEvidencePolicyInput input;
  input.gmm_health = health;

  const auto evidence = hl::build_transition_evidence(input);
  EXPECT_FALSE(evidence.local_recovery_succeeded);
  EXPECT_TRUE(evidence.local_recovery_failed);
  EXPECT_FALSE(evidence.global_recovery_failed);
  EXPECT_FALSE(evidence.emergency_global_recovery);

  health.recovery_failure_score = 0.99;
  input.gmm_health = health;
  const auto emergency = hl::build_transition_evidence(input);
  EXPECT_TRUE(emergency.global_recovery_failed);
  EXPECT_TRUE(emergency.emergency_global_recovery);
}

TEST(LocalizationEvidencePolicy, DeclaresUnavailableEmptyGmmEmergency)
{
  hl::LocalizationHealthMetrics health;
  health.component_count = 0U;
  health.represented_weight = 0.0;
  health.discarded_weight = 1.0;
  health.effective_component_count = 0.0;

  hl::LocalizationEvidencePolicyInput input;
  input.gmm_health = health;

  const auto evidence = hl::build_transition_evidence(input);
  EXPECT_FALSE(evidence.gmm_available);
  EXPECT_TRUE(evidence.gmm_health_bad);
  EXPECT_TRUE(evidence.emergency_global_recovery);
}

TEST(LocalizationEvidencePolicy, RejectsInvalidThresholdOrderingAndInputs)
{
  hl::LocalizationEvidencePolicyConfig invalid;
  invalid.good_minimum_represented_weight = 0.4;
  invalid.bad_maximum_represented_weight = 0.6;
  EXPECT_THROW(
    hl::validate_localization_evidence_policy_config(invalid),
    std::invalid_argument);

  hl::LocalizationEvidencePolicyInput invalid_time;
  invalid_time.timestamp_seconds = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(hl::build_transition_evidence(invalid_time), std::invalid_argument);

  hl::LocalizationEvidencePolicyInput invalid_particle;
  invalid_particle.particle_metrics =
    hl::ParticleBeliefMetrics{10U, 1U, 0.7, 0.1, 0.6};
  EXPECT_THROW(hl::build_transition_evidence(invalid_particle), std::invalid_argument);

  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.5},
    {{1.0, 0.0, 0.0}, 0.5}};
  hl::ParticleClusteringResult incomplete;
  incomplete.clusters = {{{0U}, 0.5}};
  EXPECT_THROW(
    hl::summarize_particle_belief(particles, incomplete),
    std::invalid_argument);
}
