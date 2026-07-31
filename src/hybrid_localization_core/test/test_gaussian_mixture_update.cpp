#include "hybrid_localization_core/gaussian_mixture_update.hpp"

#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

namespace hl = hybrid_localization;

namespace
{

constexpr double kTolerance = 1e-9;

hl::GaussianComponent make_component(
  const hl::Pose2d mean,
  const double variance,
  const double weight = 1.0)
{
  return hl::GaussianComponent{
    mean,
    {variance, 0.0, 0.0,
      0.0, variance, 0.0,
      0.0, 0.0, variance},
    weight,
    10U};
}

hl::GaussianPoseObservation make_observation(
  const hl::Pose2d mean,
  const double variance)
{
  return hl::GaussianPoseObservation{
    mean,
    {variance, 0.0, 0.0,
      0.0, variance, 0.0,
      0.0, 0.0, variance}};
}

}  // namespace

TEST(GaussianMixtureUpdate, CorrectsMeanAndReducesCovariance)
{
  const auto component = make_component({0.0, 0.0, 0.0}, 1.0);
  const auto observation = make_observation({2.0, -2.0, 1.0}, 1.0);

  const auto result = hl::update_gaussian_component(component, observation);

  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.component.mean.x, 1.0, kTolerance);
  EXPECT_NEAR(result.component.mean.y, -1.0, kTolerance);
  EXPECT_NEAR(result.component.mean.yaw, 0.5, kTolerance);
  EXPECT_NEAR(result.component.covariance[0U], 0.5, kTolerance);
  EXPECT_NEAR(result.component.covariance[4U], 0.5, kTolerance);
  EXPECT_NEAR(result.component.covariance[8U], 0.5, kTolerance);
}

TEST(GaussianMixtureUpdate, UsesWrappedYawInnovation)
{
  constexpr double degrees = std::numbers::pi / 180.0;
  const auto component = make_component({0.0, 0.0, 179.0 * degrees}, 1.0);
  const auto observation = make_observation({0.0, 0.0, -179.0 * degrees}, 1.0);

  const auto result = hl::update_gaussian_component(component, observation);

  EXPECT_NEAR(result.innovation.yaw, 2.0 * degrees, 1e-12);
  EXPECT_NEAR(std::abs(result.component.mean.yaw), std::numbers::pi, 2.0 * degrees);
}

TEST(GaussianMixtureUpdate, RejectsInnovationOutsideGate)
{
  const auto component = make_component({0.0, 0.0, 0.0}, 0.1);
  const auto observation = make_observation({10.0, 0.0, 0.0}, 0.1);

  hl::GaussianUpdateConfig config;
  config.maximum_mahalanobis_distance_squared = 9.0;
  config.likelihood_floor = 1e-6;

  const auto result = hl::update_gaussian_component(component, observation, config);

  EXPECT_FALSE(result.accepted);
  EXPECT_DOUBLE_EQ(result.component.mean.x, component.mean.x);
  EXPECT_DOUBLE_EQ(result.component.covariance[0U], component.covariance[0U]);
  EXPECT_DOUBLE_EQ(result.likelihood, config.likelihood_floor);
}

TEST(GaussianMixtureUpdate, ReweightsComponentsByLikelihood)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    make_component({0.0, 0.0, 0.0}, 0.5, 0.4),
    make_component({5.0, 0.0, 0.0}, 0.5, 0.4)};
  mixture.discarded_weight = 0.2;

  const auto observation = make_observation({0.1, 0.0, 0.0}, 0.2);
  const auto result = hl::update_gaussian_mixture(mixture, observation);

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_GT(result.mixture.components[0U].weight, result.mixture.components[1U].weight);
  EXPECT_NEAR(
    result.mixture.components[0U].weight +
    result.mixture.components[1U].weight,
    0.8,
    kTolerance);
  EXPECT_DOUBLE_EQ(result.mixture.discarded_weight, 0.2);
  EXPECT_GT(result.normalization_evidence, 0.0);
}

TEST(GaussianMixtureUpdate, PreservesMetadataAndOrdering)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    make_component({-1.0, 0.0, 0.0}, 1.0, 0.3),
    make_component({1.0, 0.0, 0.0}, 1.0, 0.7)};
  mixture.components[0U].sample_count = 4U;
  mixture.components[1U].sample_count = 8U;

  const auto result = hl::update_gaussian_mixture(
    mixture,
    make_observation({0.0, 0.0, 0.0}, 1.0));

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_EQ(result.mixture.components[0U].sample_count, 4U);
  EXPECT_EQ(result.mixture.components[1U].sample_count, 8U);
  EXPECT_LT(result.mixture.components[0U].mean.x, 0.0);
  EXPECT_GT(result.mixture.components[1U].mean.x, 0.0);
}

TEST(GaussianMixtureUpdate, SupportsSingularCovarianceThroughRegularization)
{
  const auto component = make_component({0.0, 0.0, 0.0}, 0.0);
  const auto observation = make_observation({0.0, 0.0, 0.0}, 0.0);

  hl::GaussianUpdateConfig config;
  config.innovation_covariance_regularization = 1e-9;

  const auto result = hl::update_gaussian_component(component, observation, config);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(std::isfinite(result.likelihood));
}

TEST(GaussianMixtureUpdate, RejectsInvalidInput)
{
  const auto component = make_component({0.0, 0.0, 0.0}, 1.0);
  const auto observation = make_observation({0.0, 0.0, 0.0}, 1.0);

  auto invalid_component = component;
  invalid_component.covariance[1U] = 1.0;
  EXPECT_THROW(
    hl::update_gaussian_component(invalid_component, observation),
    std::invalid_argument);

  auto invalid_observation = observation;
  invalid_observation.mean.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    hl::update_gaussian_component(component, invalid_observation),
    std::invalid_argument);

  hl::GaussianUpdateConfig invalid_config;
  invalid_config.maximum_mahalanobis_distance_squared = -1.0;
  EXPECT_THROW(
    hl::update_gaussian_component(component, observation, invalid_config),
    std::invalid_argument);

  hl::GaussianMixture invalid_mixture;
  invalid_mixture.components = {make_component({0.0, 0.0, 0.0}, 1.0, 0.5)};
  invalid_mixture.discarded_weight = 0.0;
  EXPECT_THROW(
    hl::update_gaussian_mixture(invalid_mixture, observation),
    std::invalid_argument);
}