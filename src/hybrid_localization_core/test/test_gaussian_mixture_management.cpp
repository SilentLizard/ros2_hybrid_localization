#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "hybrid_localization_core/gaussian_mixture_management.hpp"

namespace hl = hybrid_localization;
namespace
{

hl::GaussianComponent component(const double x, const double weight)
{
  hl::GaussianComponent value;
  value.mean.x = x;
  value.covariance = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  value.weight = weight;
  value.sample_count = static_cast<std::size_t>(x + 10.0);
  return value;
}

TEST(GaussianMixtureManagement, NormalizesArbitraryPositiveMass)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(1.0, 2.0),
    component(2.0, 1.0)};
  mixture.discarded_weight = 1.0;

  const auto result = hl::manage_gaussian_mixture(mixture);

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_DOUBLE_EQ(result.normalization_scale, 0.25);
  EXPECT_DOUBLE_EQ(result.mixture.components[0].weight, 0.5);
  EXPECT_DOUBLE_EQ(result.mixture.components[1].weight, 0.25);
  EXPECT_DOUBLE_EQ(result.mixture.discarded_weight, 0.25);
}

TEST(GaussianMixtureManagement, PrunesBelowThresholdIntoDiscardedMass)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(1.0, 0.70),
    component(2.0, 0.04),
    component(3.0, 0.16)};
  mixture.discarded_weight = 0.10;

  hl::GaussianMixtureManagementConfig config;
  config.minimum_component_weight = 0.05;

  const auto result = hl::manage_gaussian_mixture(mixture, config);

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_EQ(result.pruned_component_count, 1U);
  EXPECT_NEAR(result.pruned_weight, 0.04, 1e-12);
  EXPECT_NEAR(result.mixture.discarded_weight, 0.14, 1e-12);
}

TEST(GaussianMixtureManagement, KeepsLargestComponentsWhenCountIsLimited)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(1.0, 0.20),
    component(2.0, 0.50),
    component(3.0, 0.25)};
  mixture.discarded_weight = 0.05;

  hl::GaussianMixtureManagementConfig config;
  config.maximum_component_count = 2U;

  const auto result = hl::manage_gaussian_mixture(mixture, config);

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_DOUBLE_EQ(result.mixture.components[0].weight, 0.50);
  EXPECT_DOUBLE_EQ(result.mixture.components[1].weight, 0.25);
  EXPECT_NEAR(result.mixture.discarded_weight, 0.25, 1e-12);
  EXPECT_EQ(result.pruned_component_count, 1U);
}

TEST(GaussianMixtureManagement, UsesStableOrderingForEqualWeights)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(1.0, 0.40),
    component(2.0, 0.40)};
  mixture.discarded_weight = 0.20;

  const auto result = hl::manage_gaussian_mixture(mixture);

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_DOUBLE_EQ(result.mixture.components[0].mean.x, 1.0);
  EXPECT_DOUBLE_EQ(result.mixture.components[1].mean.x, 2.0);
}

TEST(GaussianMixtureManagement, AllowsAllComponentsToBeDiscarded)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(1.0, 0.20),
    component(2.0, 0.30)};
  mixture.discarded_weight = 0.50;

  hl::GaussianMixtureManagementConfig config;
  config.maximum_component_count = 0U;

  const auto result = hl::manage_gaussian_mixture(mixture, config);

  EXPECT_TRUE(result.mixture.components.empty());
  EXPECT_EQ(result.pruned_component_count, 2U);
  EXPECT_NEAR(result.pruned_weight, 0.50, 1e-12);
  EXPECT_NEAR(result.mixture.discarded_weight, 1.0, 1e-12);
}

TEST(GaussianMixtureManagement, PreservesComponentMetadata)
{
  hl::GaussianMixture mixture;
  mixture.components = {component(4.0, 0.75)};
  mixture.discarded_weight = 0.25;

  const auto result = hl::manage_gaussian_mixture(mixture);

  ASSERT_EQ(result.mixture.components.size(), 1U);
  EXPECT_DOUBLE_EQ(result.mixture.components[0].mean.x, 4.0);
  EXPECT_EQ(result.mixture.components[0].sample_count, 14U);
  EXPECT_DOUBLE_EQ(result.mixture.components[0].covariance[0], 1.0);
}

TEST(GaussianMixtureManagement, RejectsInvalidInput)
{
  hl::GaussianMixture valid;
  valid.components = {component(1.0, 1.0)};

  hl::GaussianMixtureManagementConfig invalid_threshold;
  invalid_threshold.minimum_component_weight = -0.1;
  EXPECT_THROW(
    hl::manage_gaussian_mixture(valid, invalid_threshold),
    std::invalid_argument);

  hl::GaussianMixtureManagementConfig invalid_tolerance;
  invalid_tolerance.mass_tolerance =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    hl::manage_gaussian_mixture(valid, invalid_tolerance),
    std::invalid_argument);

  hl::GaussianMixture negative_weight = valid;
  negative_weight.components[0].weight = -1.0;
  EXPECT_THROW(
    hl::manage_gaussian_mixture(negative_weight),
    std::invalid_argument);

  hl::GaussianMixture nonfinite_discarded = valid;
  nonfinite_discarded.discarded_weight =
    std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    hl::manage_gaussian_mixture(nonfinite_discarded),
    std::invalid_argument);

  hl::GaussianMixture zero_mass;
  EXPECT_THROW(
    hl::manage_gaussian_mixture(zero_mass),
    std::invalid_argument);
}

}  // namespace