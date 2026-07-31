#include "hybrid_localization_core/gaussian_mixture_prediction.hpp"

#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace hl = hybrid_localization;

namespace
{
constexpr double pi = 3.14159265358979323846;

hl::GaussianComponent make_component()
{
  hl::GaussianComponent component;
  component.mean = {1.0, 2.0, 0.0};
  component.covariance = {
    0.1, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.0, 0.0, 0.05};
  component.weight = 0.8;
  component.sample_count = 42U;
  return component;
}
}  // namespace

TEST(GaussianMixturePrediction, ComposesBodyFrameMotionIntoMean)
{
  auto component = make_component();
  component.mean.yaw = pi / 2.0;

  hl::GaussianMotionIncrement motion;
  motion.mean = {2.0, 1.0, 0.25};

  const auto predicted = hl::predict_gaussian_component(component, motion);

  EXPECT_NEAR(predicted.mean.x, 0.0, 1e-12);
  EXPECT_NEAR(predicted.mean.y, 4.0, 1e-12);
  EXPECT_NEAR(predicted.mean.yaw, pi / 2.0 + 0.25, 1e-12);
  EXPECT_DOUBLE_EQ(predicted.weight, component.weight);
  EXPECT_EQ(predicted.sample_count, component.sample_count);
}

TEST(GaussianMixturePrediction, NormalizesPredictedYaw)
{
  auto component = make_component();
  component.mean.yaw = pi - 0.1;

  hl::GaussianMotionIncrement motion;
  motion.mean.yaw = 0.3;

  const auto predicted = hl::predict_gaussian_component(component, motion);
  EXPECT_NEAR(predicted.mean.yaw, -pi + 0.2, 1e-12);
}

TEST(GaussianMixturePrediction, AddsRotatedMotionCovariance)
{
  auto component = make_component();
  component.mean.yaw = pi / 2.0;
  component.covariance.fill(0.0);

  hl::GaussianMotionIncrement motion;
  motion.covariance = {
    4.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 0.25};

  const auto predicted = hl::predict_gaussian_component(component, motion);

  EXPECT_NEAR(predicted.covariance[0], 1.0, 1e-12);
  EXPECT_NEAR(predicted.covariance[4], 4.0, 1e-12);
  EXPECT_NEAR(predicted.covariance[8], 0.25, 1e-12);
}

TEST(GaussianMixturePrediction, CouplesYawUncertaintyIntoPosition)
{
  auto component = make_component();
  component.covariance.fill(0.0);
  component.covariance[8] = 0.25;

  hl::GaussianMotionIncrement motion;
  motion.mean = {2.0, 0.0, 0.0};

  const auto predicted = hl::predict_gaussian_component(component, motion);

  EXPECT_NEAR(predicted.covariance[0], 0.0, 1e-12);
  EXPECT_NEAR(predicted.covariance[4], 1.0, 1e-12);
  EXPECT_NEAR(predicted.covariance[5], 0.5, 1e-12);
  EXPECT_NEAR(predicted.covariance[7], 0.5, 1e-12);
  EXPECT_NEAR(predicted.covariance[8], 0.25, 1e-12);
}

TEST(GaussianMixturePrediction, AppliesVarianceFloors)
{
  auto component = make_component();
  component.covariance.fill(0.0);

  hl::GaussianMotionIncrement motion;

  hl::GaussianPredictionConfig config;
  config.minimum_position_variance = 0.04;
  config.minimum_yaw_variance = 0.01;

  const auto predicted = hl::predict_gaussian_component(component, motion, config);

  EXPECT_DOUBLE_EQ(predicted.covariance[0], 0.04);
  EXPECT_DOUBLE_EQ(predicted.covariance[4], 0.04);
  EXPECT_DOUBLE_EQ(predicted.covariance[8], 0.01);
}

TEST(GaussianMixturePrediction, PredictsWholeMixtureAndPreservesMass)
{
  hl::GaussianMixture mixture;
  auto first = make_component();
  first.weight = 0.6;
  auto second = make_component();
  second.mean = {-2.0, 1.0, -0.5};
  second.weight = 0.3;
  mixture.components = {first, second};
  mixture.discarded_weight = 0.1;

  hl::GaussianMotionIncrement motion;
  motion.mean = {1.0, 0.0, 0.2};

  const auto predicted = hl::predict_gaussian_mixture(mixture, motion);

  ASSERT_EQ(predicted.components.size(), 2U);
  EXPECT_DOUBLE_EQ(predicted.components[0].weight, 0.6);
  EXPECT_DOUBLE_EQ(predicted.components[1].weight, 0.3);
  EXPECT_DOUBLE_EQ(predicted.discarded_weight, 0.1);
  EXPECT_NEAR(predicted.components[0].mean.x, 2.0, 1e-12);
}

TEST(GaussianMixturePrediction, RejectsInvalidInput)
{
  const auto component = make_component();
  hl::GaussianMotionIncrement motion;

  auto invalid_component = component;
  invalid_component.mean.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    hl::predict_gaussian_component(invalid_component, motion),
    std::invalid_argument);

  invalid_component = component;
  invalid_component.covariance[1] = 1.0;
  EXPECT_THROW(
    hl::predict_gaussian_component(invalid_component, motion),
    std::invalid_argument);

  auto invalid_motion = motion;
  invalid_motion.covariance[0] = -1.0;
  EXPECT_THROW(
    hl::predict_gaussian_component(component, invalid_motion),
    std::invalid_argument);

  hl::GaussianPredictionConfig invalid_config;
  invalid_config.minimum_yaw_variance = -1.0;
  EXPECT_THROW(
    hl::predict_gaussian_component(component, motion, invalid_config),
    std::invalid_argument);

  hl::GaussianMixture invalid_mixture;
  invalid_mixture.components = {component};
  invalid_mixture.discarded_weight = 0.1;
  EXPECT_THROW(
    hl::predict_gaussian_mixture(invalid_mixture, motion),
    std::invalid_argument);
}