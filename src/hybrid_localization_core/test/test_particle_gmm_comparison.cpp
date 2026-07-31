#include <gtest/gtest.h>

#include "hybrid_localization_core/particle_gmm_comparison.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hl = hybrid_localization;

namespace
{

hl::GaussianComponent component(
  const double x,
  const double y,
  const double yaw,
  const double variance,
  const double weight)
{
  hl::GaussianComponent result{};
  result.mean = {x, y, yaw};
  result.covariance = {variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance};
  result.weight = weight;
  result.sample_count = 10U;
  return result;
}

}  // namespace

TEST(ParticleGmmComparison, ReportsAgreementForMatchingBeliefs)
{
  const std::vector<hl::WeightedParticle> particles{
    {{-1.0, 0.0, 0.0}, 0.5},
    {{1.0, 0.0, 0.0}, 0.5}};
  hl::GaussianMixture mixture;
  mixture.components = {component(0.0, 0.0, 0.0, 1.0, 1.0)};

  const auto result = hl::compare_particle_and_gmm_beliefs(particles, mixture);

  EXPECT_NEAR(result.position_difference, 0.0, 1e-12);
  EXPECT_NEAR(result.absolute_yaw_difference, 0.0, 1e-12);
  EXPECT_NEAR(result.particle_mean.x, 0.0, 1e-12);
  EXPECT_NEAR(result.gmm_mean.x, 0.0, 1e-12);
  EXPECT_EQ(result.dominant_component_index, 0U);
}

TEST(ParticleGmmComparison, UsesWrappedYawDifference)
{
  const std::vector<hl::WeightedParticle> particles{{{0.0, 0.0, 3.13}, 1.0}};
  hl::GaussianMixture mixture;
  mixture.components = {component(0.0, 0.0, -3.13, 0.1, 1.0)};

  const auto result = hl::compare_particle_and_gmm_beliefs(particles, mixture);

  EXPECT_LT(result.absolute_yaw_difference, 0.03);
  EXPECT_LT(result.dominant_component_absolute_yaw_difference, 0.03);
}

TEST(ParticleGmmComparison, MomentMatchesMultimodalGmm)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.25},
    {{2.0, 0.0, 0.0}, 0.75}};
  hl::GaussianMixture mixture;
  mixture.components = {
    component(0.0, 0.0, 0.0, 0.1, 0.25),
    component(2.0, 0.0, 0.0, 0.1, 0.75)};

  const auto result = hl::compare_particle_and_gmm_beliefs(particles, mixture);

  EXPECT_NEAR(result.gmm_mean.x, 1.5, 1e-12);
  EXPECT_NEAR(result.gmm_covariance[0], 0.85, 1e-12);
  EXPECT_EQ(result.dominant_component_index, 1U);
}

TEST(ParticleGmmComparison, MeasuresParticleSupportAcrossAllComponents)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.4},
    {{10.0, 0.0, 0.0}, 0.4},
    {{30.0, 0.0, 0.0}, 0.2}};
  hl::GaussianMixture mixture;
  mixture.components = {
    component(0.0, 0.0, 0.0, 1.0, 0.5),
    component(10.0, 0.0, 0.0, 1.0, 0.5)};

  const auto result = hl::compare_particle_and_gmm_beliefs(particles, mixture);

  EXPECT_NEAR(result.particle_mass_supported_by_gmm, 0.8, 1e-12);
}

TEST(ParticleGmmComparison, ReportsDominantAndAggregateDisagreement)
{
  const std::vector<hl::WeightedParticle> particles{{{5.0, 0.0, 0.0}, 1.0}};
  hl::GaussianMixture mixture;
  mixture.components = {
    component(0.0, 0.0, 0.0, 1.0, 0.8),
    component(5.0, 0.0, 0.0, 1.0, 0.2)};

  const auto result = hl::compare_particle_and_gmm_beliefs(particles, mixture);

  EXPECT_EQ(result.dominant_component_index, 0U);
  EXPECT_NEAR(result.dominant_component_position_difference, 5.0, 1e-12);
  EXPECT_NEAR(result.position_difference, 4.0, 1e-12);
}

TEST(ParticleGmmComparison, PreservesReportedGmmMass)
{
  const std::vector<hl::WeightedParticle> particles{{{0.0, 0.0, 0.0}, 1.0}};
  hl::GaussianMixture mixture;
  mixture.components = {component(0.0, 0.0, 0.0, 1.0, 0.7)};
  mixture.discarded_weight = 0.3;

  const auto result = hl::compare_particle_and_gmm_beliefs(particles, mixture);

  EXPECT_NEAR(result.represented_gmm_weight, 0.7, 1e-12);
  EXPECT_NEAR(result.discarded_gmm_weight, 0.3, 1e-12);
}

TEST(ParticleGmmComparison, RejectsInvalidInput)
{
  const std::vector<hl::WeightedParticle> particles{{{0.0, 0.0, 0.0}, 1.0}};
  hl::GaussianMixture valid_mixture;
  valid_mixture.components = {component(0.0, 0.0, 0.0, 1.0, 1.0)};

  EXPECT_THROW(
    hl::compare_particle_and_gmm_beliefs({}, valid_mixture),
    std::invalid_argument);

  auto empty_mixture = hl::GaussianMixture{};
  empty_mixture.discarded_weight = 1.0;
  EXPECT_THROW(
    hl::compare_particle_and_gmm_beliefs(particles, empty_mixture),
    std::invalid_argument);

  auto invalid_mass = valid_mixture;
  invalid_mass.discarded_weight = 0.2;
  EXPECT_THROW(
    hl::compare_particle_and_gmm_beliefs(particles, invalid_mass),
    std::invalid_argument);

  auto invalid_covariance = valid_mixture;
  invalid_covariance.components[0].covariance[1] = 1.0;
  EXPECT_THROW(
    hl::compare_particle_and_gmm_beliefs(particles, invalid_covariance),
    std::invalid_argument);

  hl::ParticleGmmComparisonConfig invalid_config;
  invalid_config.covariance_regularization = 0.0;
  EXPECT_THROW(
    hl::compare_particle_and_gmm_beliefs(particles, valid_mixture, invalid_config),
    std::invalid_argument);
}
