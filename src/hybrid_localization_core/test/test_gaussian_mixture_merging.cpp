#include "hybrid_localization_core/gaussian_mixture_merging.hpp"

#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

namespace hl = hybrid_localization;

namespace
{

hl::GaussianComponent component(
  const double x,
  const double y,
  const double yaw,
  const double variance,
  const double weight,
  const std::size_t sample_count)
{
  return hl::GaussianComponent{
    .mean = {x, y, yaw},
    .covariance = {
      variance, 0.0, 0.0,
      0.0, variance, 0.0,
      0.0, 0.0, variance},
    .weight = weight,
    .sample_count = sample_count};
}

}  // namespace

TEST(GaussianMixtureMerging, MergesCompatibleComponentsAndPreservesMass)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(0.0, 0.0, 0.0, 1.0, 0.3, 3U),
    component(2.0, 0.0, 0.0, 1.0, 0.5, 5U)};
  mixture.discarded_weight = 0.2;

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = 3.0;
  config.maximum_yaw_difference = 0.1;

  const auto result = hl::merge_gaussian_mixture_components(mixture, config);

  ASSERT_EQ(result.merge_count, 1U);
  ASSERT_EQ(result.mixture.components.size(), 1U);
  EXPECT_NEAR(result.mixture.components[0].weight, 0.8, 1e-12);
  EXPECT_NEAR(result.mixture.discarded_weight, 0.2, 1e-12);
  EXPECT_EQ(result.mixture.components[0].sample_count, 8U);
  EXPECT_NEAR(result.mixture.components[0].mean.x, 1.25, 1e-12);
}

TEST(GaussianMixtureMerging, PreservesFirstTwoMoments)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(-1.0, 0.0, 0.0, 0.5, 0.25, 2U),
    component(1.0, 0.0, 0.0, 0.5, 0.75, 6U)};

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = 10.0;

  const auto result = hl::merge_gaussian_mixture_components(mixture, config);
  const auto & merged = result.mixture.components.at(0);

  EXPECT_NEAR(merged.mean.x, 0.5, 1e-12);
  EXPECT_NEAR(merged.covariance[0], 1.25, 1e-12);
  EXPECT_NEAR(merged.covariance[4], 0.5, 1e-12);
  EXPECT_NEAR(merged.covariance[8], 0.5, 1e-12);
}

TEST(GaussianMixtureMerging, UsesCircularYawMeanAcrossWraparound)
{
  constexpr double degrees = std::numbers::pi / 180.0;
  hl::GaussianMixture mixture;
  mixture.components = {
    component(0.0, 0.0, 179.0 * degrees, 0.1, 0.5, 1U),
    component(0.0, 0.0, -179.0 * degrees, 0.1, 0.5, 1U)};

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = 1.0;
  config.maximum_yaw_difference = 5.0 * degrees;

  const auto result = hl::merge_gaussian_mixture_components(mixture, config);
  ASSERT_EQ(result.mixture.components.size(), 1U);
  EXPECT_NEAR(
    std::abs(result.mixture.components[0].mean.yaw),
    std::numbers::pi,
    1e-12);
}

TEST(GaussianMixtureMerging, RespectsYawCompatibilityLimit)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(0.0, 0.0, 0.0, 10.0, 0.5, 1U),
    component(0.0, 0.0, 1.0, 10.0, 0.5, 1U)};

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = 100.0;
  config.maximum_yaw_difference = 0.5;

  const auto result = hl::merge_gaussian_mixture_components(mixture, config);
  EXPECT_EQ(result.merge_count, 0U);
  EXPECT_EQ(result.mixture.components.size(), 2U);
}

TEST(GaussianMixtureMerging, SelectsClosestPairDeterministically)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(0.0, 0.0, 0.0, 1.0, 0.2, 1U),
    component(0.1, 0.0, 0.0, 1.0, 0.3, 1U),
    component(3.0, 0.0, 0.0, 1.0, 0.5, 1U)};

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = 0.1;

  const auto result = hl::merge_gaussian_mixture_components(mixture, config);
  ASSERT_EQ(result.merge_count, 1U);
  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_NEAR(result.mixture.components[0].weight, 0.5, 1e-12);
  EXPECT_NEAR(result.mixture.components[1].weight, 0.5, 1e-12);  
  // The merged pair occupies the first position. Both resulting components
  // have equal weights, so stable sorting preserves their pre-sort order.
  EXPECT_NEAR(result.mixture.components[0].mean.x, 0.06, 1e-12);
  EXPECT_NEAR(result.mixture.components[1].mean.x, 3.0, 1e-12);
  EXPECT_EQ(result.mixture.components[0].sample_count, 2U);
  EXPECT_EQ(result.mixture.components[1].sample_count, 1U);
}

TEST(GaussianMixtureMerging, LeavesIncompatibleComponentsUnchanged)
{
  hl::GaussianMixture mixture;
  mixture.components = {
    component(-10.0, 0.0, 0.0, 0.1, 0.4, 4U),
    component(10.0, 0.0, 0.0, 0.1, 0.6, 6U)};

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = 1.0;

  const auto result = hl::merge_gaussian_mixture_components(mixture, config);
  EXPECT_EQ(result.merge_count, 0U);
  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_NEAR(result.mixture.components[0].weight, 0.6, 1e-12);
  EXPECT_NEAR(result.mixture.components[1].weight, 0.4, 1e-12);
}

TEST(GaussianMixtureMerging, RejectsInvalidInput)
{
  hl::GaussianMixture valid;
  valid.components = {component(0.0, 0.0, 0.0, 1.0, 1.0, 1U)};

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = -1.0;
  EXPECT_THROW(
    hl::merge_gaussian_mixture_components(valid, config),
    std::invalid_argument);

  config = {};
  config.covariance_regularization = -1.0;
  EXPECT_THROW(
    hl::merge_gaussian_mixture_components(valid, config),
    std::invalid_argument);

  auto invalid_mass = valid;
  invalid_mass.components[0].weight = 0.9;
  EXPECT_THROW(
    hl::merge_gaussian_mixture_components(invalid_mass),
    std::invalid_argument);

  auto asymmetric = valid;
  asymmetric.components[0].covariance[1] = 1.0;
  EXPECT_THROW(
    hl::merge_gaussian_mixture_components(asymmetric),
    std::invalid_argument);

  auto zero_weight = valid;
  zero_weight.components[0].weight = 0.0;
  zero_weight.discarded_weight = 1.0;
  EXPECT_THROW(
    hl::merge_gaussian_mixture_components(zero_weight),
    std::invalid_argument);

  auto non_finite = valid;
  non_finite.components[0].mean.x =
    std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    hl::merge_gaussian_mixture_components(non_finite),
    std::invalid_argument);
}