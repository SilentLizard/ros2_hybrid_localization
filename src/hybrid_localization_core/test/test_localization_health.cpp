#include "hybrid_localization_core/localization_health.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace hl = hybrid_localization;

namespace
{

hl::GaussianComponent component(
  const double weight,
  const double x_variance,
  const double y_variance,
  const double yaw_variance)
{
  return hl::GaussianComponent{
    {},
    {x_variance, 0.0, 0.0,
      0.0, y_variance, 0.0,
      0.0, 0.0, yaw_variance},
    weight,
    1U};
}

hl::GaussianMixture mixture()
{
  return hl::GaussianMixture{
    {component(0.6, 1.0, 2.0, 0.3), component(0.2, 4.0, 6.0, 0.9)},
    0.2};
}

}  // namespace

TEST(LocalizationHealth, SummarizesMixtureMassAndAmbiguity)
{
  const auto metrics = hl::evaluate_localization_health(mixture());

  EXPECT_EQ(metrics.component_count, 2U);
  EXPECT_NEAR(metrics.represented_weight, 0.8, 1e-12);
  EXPECT_NEAR(metrics.discarded_weight, 0.2, 1e-12);
  EXPECT_NEAR(metrics.dominant_component_weight, 0.6, 1e-12);
  EXPECT_GT(metrics.normalized_mixture_entropy, 0.0);
  EXPECT_LT(metrics.normalized_mixture_entropy, 1.0);
  EXPECT_GT(metrics.effective_component_count, 1.0);
  EXPECT_LT(metrics.effective_component_count, 2.0);
}

TEST(LocalizationHealth, ComputesConditionedCovarianceSummaries)
{
  const auto metrics = hl::evaluate_localization_health(mixture());

  EXPECT_NEAR(metrics.weighted_position_variance, 4.75, 1e-12);
  EXPECT_NEAR(metrics.weighted_yaw_variance, 0.45, 1e-12);
  EXPECT_NEAR(metrics.maximum_position_variance, 10.0, 1e-12);
  EXPECT_NEAR(metrics.maximum_yaw_variance, 0.9, 1e-12);
}

TEST(LocalizationHealth, SummarizesMeasurementConsistency)
{
  const auto belief = mixture();
  hl::GaussianMixtureUpdate update;
  update.mixture = belief;
  update.normalization_evidence = 0.25;
  update.component_updates = {
    {.mahalanobis_distance_squared = 1.0, .likelihood = 0.5, .accepted = true},
    {.mahalanobis_distance_squared = 9.0, .likelihood = 0.1, .accepted = false}};

  hl::LocalizationHealthEvidence evidence;
  evidence.measurement_update = &update;
  const auto metrics = hl::evaluate_localization_health(belief, evidence);

  EXPECT_TRUE(metrics.has_measurement_update);
  EXPECT_NEAR(metrics.accepted_component_fraction, 0.5, 1e-12);
  EXPECT_NEAR(metrics.accepted_component_weight_fraction, 0.75, 1e-12);
  EXPECT_NEAR(metrics.weighted_mahalanobis_distance_squared, 3.0, 1e-12);
  EXPECT_NEAR(metrics.maximum_mahalanobis_distance_squared, 9.0, 1e-12);
  EXPECT_NEAR(metrics.normalization_evidence, 0.25, 1e-12);
}

TEST(LocalizationHealth, SummarizesGaussianFitQuality)
{
  const auto belief = mixture();
  const std::array<hl::GaussianFitQuality, 2> quality{{
    {1.0, 2.0, 0.9},
    {3.0, 5.0, 0.4}}};

  hl::LocalizationHealthEvidence evidence;
  evidence.fit_quality = quality;
  const auto metrics = hl::evaluate_localization_health(belief, evidence);

  EXPECT_TRUE(metrics.has_fit_quality);
  EXPECT_NEAR(metrics.weighted_fit_mean_mahalanobis_distance, 1.5, 1e-12);
  EXPECT_NEAR(metrics.maximum_fit_mahalanobis_distance, 5.0, 1e-12);
  EXPECT_NEAR(metrics.minimum_angular_resultant_length, 0.4, 1e-12);
}

TEST(LocalizationHealth, ReportsRecoveryFailureEvidence)
{
  hl::LocalizationHealthEvidence evidence;
  evidence.recovery_failure_score = 0.7;

  const auto metrics = hl::evaluate_localization_health(mixture(), evidence);
  EXPECT_TRUE(metrics.has_recovery_failure_score);
  EXPECT_NEAR(metrics.recovery_failure_score, 0.7, 1e-12);
}

TEST(LocalizationHealth, SupportsFullyDiscardedMixture)
{
  const hl::GaussianMixture belief{{}, 1.0};
  const auto metrics = hl::evaluate_localization_health(belief);

  EXPECT_EQ(metrics.component_count, 0U);
  EXPECT_DOUBLE_EQ(metrics.represented_weight, 0.0);
  EXPECT_DOUBLE_EQ(metrics.effective_component_count, 0.0);
  EXPECT_DOUBLE_EQ(metrics.weighted_position_variance, 0.0);
}

TEST(LocalizationHealth, RejectsInvalidInputAndMismatchedEvidence)
{
  auto invalid = mixture();
  invalid.discarded_weight = 0.1;
  EXPECT_THROW(hl::evaluate_localization_health(invalid), std::invalid_argument);

  auto invalid_covariance = mixture();
  invalid_covariance.components[0U].covariance[0U] = -1.0;
  EXPECT_THROW(
    hl::evaluate_localization_health(invalid_covariance),
    std::invalid_argument);

  const auto belief = mixture();
  hl::GaussianMixtureUpdate update;
  update.mixture = belief;
  update.component_updates.resize(1U);
  hl::LocalizationHealthEvidence update_evidence;
  update_evidence.measurement_update = &update;
  EXPECT_THROW(
    hl::evaluate_localization_health(belief, update_evidence),
    std::invalid_argument);

  const std::array<hl::GaussianFitQuality, 1> quality{};
  hl::LocalizationHealthEvidence fit_evidence;
  fit_evidence.fit_quality = quality;
  EXPECT_THROW(
    hl::evaluate_localization_health(belief, fit_evidence),
    std::invalid_argument);

  hl::LocalizationHealthEvidence recovery_evidence;
  recovery_evidence.recovery_failure_score =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    hl::evaluate_localization_health(belief, recovery_evidence),
    std::invalid_argument);
}
