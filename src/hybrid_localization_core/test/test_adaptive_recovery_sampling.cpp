#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_localization_core/adaptive_recovery_sampling.hpp"

namespace hl = hybrid_localization;

namespace
{

hl::GaussianComponent point_component(
  const double x,
  const double weight)
{
  return {
    {x, 0.0, 0.0},
    {0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0},
    weight,
    1U};
}

hl::OccupancyGridView free_grid(const std::vector<std::int8_t> & cells)
{
  return {cells.size(), 1U, 1.0, {}, cells};
}

}  // namespace

TEST(AdaptiveRecoverySampling, ComputesNormalizedMixtureEntropy)
{
  EXPECT_DOUBLE_EQ(
    hl::normalized_mixture_entropy(
      {{{point_component(0.0, 1.0)}}, 0.0}),
    0.0);

  const hl::GaussianMixture equal{
    {point_component(0.0, 0.5), point_component(1.0, 0.5)},
    0.0};
  EXPECT_NEAR(hl::normalized_mixture_entropy(equal), 1.0, 1e-12);

  const hl::GaussianMixture unequal{
    {point_component(0.0, 0.9), point_component(1.0, 0.1)},
    0.0};
  EXPECT_GT(hl::normalized_mixture_entropy(unequal), 0.0);
  EXPECT_LT(hl::normalized_mixture_entropy(unequal), 1.0);
}

TEST(AdaptiveRecoverySampling, CombinesExactLocalAndGlobalCounts)
{
  const hl::GaussianMixture mixture{{point_component(0.0, 1.0)}, 0.0};
  const std::vector<std::int8_t> cells{0};

  hl::AdaptiveRecoverySamplingConfig config;
  config.particle_count = 10U;
  config.base_global_fraction = 0.3;
  config.random_seed = 42U;

  const auto result = hl::sample_adaptive_recovery_particles(
    mixture, free_grid(cells), {}, config);

  EXPECT_EQ(result.local_particle_count, 7U);
  EXPECT_EQ(result.global_particle_count, 3U);
  EXPECT_DOUBLE_EQ(result.global_fraction, 0.3);
  ASSERT_EQ(result.particles.size(), 10U);

  const double total_weight = std::accumulate(
    result.particles.begin(), result.particles.end(), 0.0,
    [](const double total, const hl::WeightedParticle & particle) {
      return total + particle.weight;
    });
  EXPECT_NEAR(total_weight, 1.0, 1e-12);
}

TEST(AdaptiveRecoverySampling, AdaptsFromDiscardedEntropyAndFailureEvidence)
{
  const hl::GaussianMixture mixture{
    {point_component(0.0, 0.4), point_component(2.0, 0.4)},
    0.2};
  const std::vector<std::int8_t> cells{0};

  hl::AdaptiveRecoverySamplingConfig config;
  config.particle_count = 100U;
  config.base_global_fraction = 0.1;
  config.discarded_weight_gain = 0.5;
  config.mixture_entropy_gain = 0.2;
  config.failure_score_gain = 0.3;

  const auto result = hl::sample_adaptive_recovery_particles(
    mixture, free_grid(cells), {0.5}, config);

  // 0.1 + 0.5*0.2 + 0.2*1.0 + 0.3*0.5 = 0.55
  EXPECT_EQ(result.global_particle_count, 55U);
  EXPECT_EQ(result.local_particle_count, 45U);
  EXPECT_NEAR(result.normalized_mixture_entropy, 1.0, 1e-12);
}

TEST(AdaptiveRecoverySampling, ClampsGlobalFraction)
{
  const hl::GaussianMixture mixture{{point_component(0.0, 1.0)}, 0.0};
  const std::vector<std::int8_t> cells{0};

  hl::AdaptiveRecoverySamplingConfig minimum_config;
  minimum_config.particle_count = 20U;
  minimum_config.base_global_fraction = 0.0;
  minimum_config.minimum_global_fraction = 0.25;
  const auto minimum_result = hl::sample_adaptive_recovery_particles(
    mixture, free_grid(cells), {}, minimum_config);
  EXPECT_EQ(minimum_result.global_particle_count, 5U);

  hl::AdaptiveRecoverySamplingConfig maximum_config;
  maximum_config.particle_count = 20U;
  maximum_config.base_global_fraction = 0.8;
  maximum_config.maximum_global_fraction = 0.4;
  const auto maximum_result = hl::sample_adaptive_recovery_particles(
    mixture, free_grid(cells), {}, maximum_config);
  EXPECT_EQ(maximum_result.global_particle_count, 8U);
}

TEST(AdaptiveRecoverySampling, SupportsPureLocalAndPureGlobalRecovery)
{
  const std::vector<std::int8_t> cells{0};

  hl::AdaptiveRecoverySamplingConfig local_config;
  local_config.particle_count = 8U;
  local_config.base_global_fraction = 0.0;
  const auto local = hl::sample_adaptive_recovery_particles(
    {{point_component(3.0, 1.0)}, 0.0},
    free_grid(cells), {}, local_config);
  EXPECT_EQ(local.local_particle_count, 8U);
  EXPECT_EQ(local.global_particle_count, 0U);
  for (const auto & particle : local.particles) {
    EXPECT_DOUBLE_EQ(particle.pose.x, 3.0);
  }

  hl::AdaptiveRecoverySamplingConfig global_config;
  global_config.particle_count = 8U;
  global_config.base_global_fraction = 0.0;
  const auto global = hl::sample_adaptive_recovery_particles(
    {{}, 1.0}, free_grid(cells), {}, global_config);
  EXPECT_EQ(global.local_particle_count, 0U);
  EXPECT_EQ(global.global_particle_count, 8U);
}

TEST(AdaptiveRecoverySampling, UsesDeterministicSeed)
{
  const hl::GaussianMixture mixture{{point_component(0.0, 1.0)}, 0.0};
  const std::vector<std::int8_t> cells{0, 0};

  hl::AdaptiveRecoverySamplingConfig config;
  config.particle_count = 20U;
  config.base_global_fraction = 0.5;
  config.covariance_inflation_factor = 2.0;
  config.random_seed = 1234U;

  const auto first = hl::sample_adaptive_recovery_particles(
    mixture, free_grid(cells), {}, config);
  const auto second = hl::sample_adaptive_recovery_particles(
    mixture, free_grid(cells), {}, config);

  ASSERT_EQ(first.particles.size(), second.particles.size());
  for (std::size_t i = 0U; i < first.particles.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.particles[i].pose.x, second.particles[i].pose.x);
    EXPECT_DOUBLE_EQ(first.particles[i].pose.y, second.particles[i].pose.y);
    EXPECT_DOUBLE_EQ(first.particles[i].pose.yaw, second.particles[i].pose.yaw);
    EXPECT_DOUBLE_EQ(first.particles[i].weight, second.particles[i].weight);
  }
}

TEST(AdaptiveRecoverySampling, RejectsInvalidConfigurationAndSignals)
{
  const hl::GaussianMixture mixture{{point_component(0.0, 1.0)}, 0.0};
  const std::vector<std::int8_t> cells{0};
  const auto grid = free_grid(cells);

  hl::AdaptiveRecoverySamplingConfig config;
  config.particle_count = 0U;
  EXPECT_THROW(
    hl::sample_adaptive_recovery_particles(mixture, grid, {}, config),
    std::invalid_argument);

  config.particle_count = 10U;
  config.minimum_global_fraction = 0.8;
  config.maximum_global_fraction = 0.2;
  EXPECT_THROW(
    hl::sample_adaptive_recovery_particles(mixture, grid, {}, config),
    std::invalid_argument);

  config.minimum_global_fraction = 0.0;
  config.maximum_global_fraction = 1.0;
  config.discarded_weight_gain = -1.0;
  EXPECT_THROW(
    hl::sample_adaptive_recovery_particles(mixture, grid, {}, config),
    std::invalid_argument);

  config.discarded_weight_gain = 0.0;
  EXPECT_THROW(
    hl::sample_adaptive_recovery_particles(mixture, grid, {1.1}, config),
    std::invalid_argument);
}