#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_localization_core/recovery_sampling.hpp"

namespace hl = hybrid_localization;

namespace
{

hl::GaussianComponent make_component(
  const hl::Pose2d mean,
  const double variance,
  const double weight)
{
  return {
    mean,
    {
      variance, 0.0, 0.0,
      0.0, variance, 0.0,
      0.0, 0.0, variance
    },
    weight,
    10U
  };
}

}  // namespace

TEST(RecoverySampling, SamplesExactCountWithNormalizedWeights)
{
  hl::GaussianMixture mixture;
  mixture.components.push_back(
    make_component({1.0, 2.0, 0.5}, 0.0, 1.0));

  const auto particles = hl::sample_recovery_particles(
    mixture,
    {25U, 1.0, 42U, 0U});

  ASSERT_EQ(particles.size(), 25U);

  double weight_sum = 0.0;
  for (const auto & particle : particles) {
    EXPECT_DOUBLE_EQ(particle.pose.x, 1.0);
    EXPECT_DOUBLE_EQ(particle.pose.y, 2.0);
    EXPECT_DOUBLE_EQ(particle.pose.yaw, 0.5);
    EXPECT_DOUBLE_EQ(particle.weight, 1.0 / 25.0);
    weight_sum += particle.weight;
  }

  EXPECT_NEAR(weight_sum, 1.0, 1e-12);
}

TEST(RecoverySampling, AllocatesSamplesByRenormalizedRetainedWeight)
{
  hl::GaussianMixture mixture;
  mixture.components.push_back(
    make_component({0.0, 0.0, 0.0}, 0.0, 0.6));
  mixture.components.push_back(
    make_component({10.0, 0.0, 0.0}, 0.0, 0.3));
  mixture.discarded_weight = 0.1;

  const auto particles = hl::sample_recovery_particles(
    mixture,
    {9U, 1.0, 7U, 0U});

  std::size_t first_count = 0U;
  std::size_t second_count = 0U;

  for (const auto & particle : particles) {
    if (particle.pose.x == 0.0) {
      ++first_count;
    } else if (particle.pose.x == 10.0) {
      ++second_count;
    }
  }

  EXPECT_EQ(first_count, 6U);
  EXPECT_EQ(second_count, 3U);
}

TEST(RecoverySampling, UsesDeterministicSeed)
{
  hl::GaussianMixture mixture;
  mixture.components.push_back(
    make_component({0.0, 0.0, 0.0}, 1.0, 1.0));

  const hl::RecoverySamplingConfig config{20U, 1.0, 12345U, 0U};

  const auto first = hl::sample_recovery_particles(mixture, config);
  const auto second = hl::sample_recovery_particles(mixture, config);

  ASSERT_EQ(first.size(), second.size());

  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_DOUBLE_EQ(first[index].pose.x, second[index].pose.x);
    EXPECT_DOUBLE_EQ(first[index].pose.y, second[index].pose.y);
    EXPECT_DOUBLE_EQ(first[index].pose.yaw, second[index].pose.yaw);
    EXPECT_DOUBLE_EQ(first[index].weight, second[index].weight);
  }
}

TEST(RecoverySampling, CovarianceInflationChangesSamples)
{
  hl::GaussianMixture mixture;
  mixture.components.push_back(
    make_component({0.0, 0.0, 0.0}, 1.0, 1.0));

  const auto normal = hl::sample_recovery_particles(
    mixture,
    {1U, 1.0, 77U, 0U});
  const auto inflated = hl::sample_recovery_particles(
    mixture,
    {1U, 4.0, 77U, 0U});

  ASSERT_EQ(normal.size(), 1U);
  ASSERT_EQ(inflated.size(), 1U);

  EXPECT_NEAR(inflated[0].pose.x, 2.0 * normal[0].pose.x, 1e-12);
  EXPECT_NEAR(inflated[0].pose.y, 2.0 * normal[0].pose.y, 1e-12);
}

TEST(RecoverySampling, NormalizesSampledYaw)
{
  hl::GaussianMixture mixture;
  mixture.components.push_back(
    make_component(
      {0.0, 0.0, std::numbers::pi - 0.01},
      1.0,
      1.0));

  const auto particles = hl::sample_recovery_particles(
    mixture,
    {100U, 2.0, 99U, 0U});

  for (const auto & particle : particles) {
    EXPECT_GE(particle.pose.yaw, -std::numbers::pi);
    EXPECT_LT(particle.pose.yaw, std::numbers::pi);
  }
}

TEST(RecoverySampling, RespectsMinimumSamplesPerComponent)
{
  hl::GaussianMixture mixture;
  mixture.components.push_back(
    make_component({0.0, 0.0, 0.0}, 0.0, 0.99));
  mixture.components.push_back(
    make_component({10.0, 0.0, 0.0}, 0.0, 0.01));

  const auto particles = hl::sample_recovery_particles(
    mixture,
    {10U, 1.0, 1U, 2U});

  std::size_t second_count = 0U;
  for (const auto & particle : particles) {
    if (particle.pose.x == 10.0) {
      ++second_count;
    }
  }

  EXPECT_GE(second_count, 2U);
}

TEST(RecoverySampling, RejectsInvalidInput)
{
  hl::GaussianMixture valid_mixture;
  valid_mixture.components.push_back(
    make_component({0.0, 0.0, 0.0}, 1.0, 1.0));

  EXPECT_THROW(
    static_cast<void>(hl::sample_recovery_particles(
      valid_mixture,
      {0U, 1.0, 0U, 0U})),
    std::invalid_argument);

  EXPECT_THROW(
    static_cast<void>(hl::sample_recovery_particles(
      valid_mixture,
      {10U, 0.0, 0U, 0U})),
    std::invalid_argument);

  EXPECT_THROW(
    static_cast<void>(hl::sample_recovery_particles(
      {},
      {10U, 1.0, 0U, 0U})),
    std::invalid_argument);

  hl::GaussianMixture invalid_covariance = valid_mixture;
  invalid_covariance.components[0].covariance[0] = -1.0;

  EXPECT_THROW(
    static_cast<void>(hl::sample_recovery_particles(
      invalid_covariance,
      {10U, 1.0, 0U, 0U})),
    std::invalid_argument);

  hl::GaussianMixture two_components;
  two_components.components.push_back(
    make_component({0.0, 0.0, 0.0}, 1.0, 0.5));
  two_components.components.push_back(
    make_component({1.0, 0.0, 0.0}, 1.0, 0.5));

  EXPECT_THROW(
    static_cast<void>(hl::sample_recovery_particles(
      two_components,
      {3U, 1.0, 0U, 2U})),
    std::invalid_argument);
}