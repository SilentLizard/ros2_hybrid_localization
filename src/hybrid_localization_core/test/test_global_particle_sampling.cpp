#include "hybrid_localization_core/global_particle_sampling.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace hl = hybrid_localization;

TEST(GlobalParticleSampling, SamplesExactCountWithNormalizedWeights)
{
  const std::array<std::int8_t, 4> cells{0, 100, 0, 100};
  const hl::OccupancyGridView grid{2U, 2U, 1.0, {}, cells};
  const hl::GlobalParticleSamplingConfig config{50U, 0, hl::UnknownCellPolicy::exclude, 7U};

  const auto particles = hl::sample_global_particles(grid, config);

  ASSERT_EQ(particles.size(), 50U);

  const double weight_sum = std::accumulate(
    particles.begin(),
    particles.end(),
    0.0,
    [](const double total, const hl::WeightedParticle & particle)
    {
      return total + particle.weight;
    });

  EXPECT_NEAR(weight_sum, 1.0, 1e-12);

  for (const auto & particle : particles) {
    EXPECT_DOUBLE_EQ(particle.weight, 1.0 / 50.0);
    EXPECT_GE(particle.pose.yaw, -std::numbers::pi);
    EXPECT_LT(particle.pose.yaw, std::numbers::pi);

    const bool in_lower_left =
      particle.pose.x >= 0.0 && particle.pose.x < 1.0 &&
      particle.pose.y >= 0.0 && particle.pose.y < 1.0;
    const bool in_upper_left =
      particle.pose.x >= 0.0 && particle.pose.x < 1.0 &&
      particle.pose.y >= 1.0 && particle.pose.y < 2.0;

    EXPECT_TRUE(in_lower_left || in_upper_left);
  }
}

TEST(GlobalParticleSampling, AppliesRotatedMapOrigin)
{
  const std::array<std::int8_t, 1> cells{0};
  const hl::OccupancyGridView grid{
    1U,
    1U,
    2.0,
    {10.0, 20.0, std::numbers::pi / 2.0},
    cells};
  const hl::GlobalParticleSamplingConfig config{20U, 0, hl::UnknownCellPolicy::exclude, 11U};

  const auto particles = hl::sample_global_particles(grid, config);

  for (const auto & particle : particles) {
    EXPECT_GT(particle.pose.x, 8.0);
    EXPECT_LE(particle.pose.x, 10.0);
    EXPECT_GE(particle.pose.y, 20.0);
    EXPECT_LT(particle.pose.y, 22.0);
  }
}

TEST(GlobalParticleSampling, AppliesOccupancyThreshold)
{
  const std::array<std::int8_t, 3> cells{0, 20, 21};
  const hl::OccupancyGridView grid{3U, 1U, 1.0, {}, cells};
  const hl::GlobalParticleSamplingConfig config{100U, 20, hl::UnknownCellPolicy::exclude, 3U};

  const auto particles = hl::sample_global_particles(grid, config);

  for (const auto & particle : particles) {
    EXPECT_GE(particle.pose.x, 0.0);
    EXPECT_LT(particle.pose.x, 2.0);
  }
}

TEST(GlobalParticleSampling, SupportsExplicitUnknownCellPolicy)
{
  const std::array<std::int8_t, 2> cells{-1, 100};
  const hl::OccupancyGridView grid{2U, 1U, 1.0, {}, cells};

  EXPECT_THROW(
    hl::sample_global_particles(
      grid,
      {10U, 0, hl::UnknownCellPolicy::exclude, 5U}),
    std::invalid_argument);

  const auto particles = hl::sample_global_particles(
    grid,
    {10U, 0, hl::UnknownCellPolicy::include, 5U});

  for (const auto & particle : particles) {
    EXPECT_GE(particle.pose.x, 0.0);
    EXPECT_LT(particle.pose.x, 1.0);
  }
}

TEST(GlobalParticleSampling, UsesDeterministicSeed)
{
  const std::array<std::int8_t, 4> cells{0, 0, 0, 0};
  const hl::OccupancyGridView grid{2U, 2U, 0.5, {-1.0, 2.0, 0.2}, cells};
  const hl::GlobalParticleSamplingConfig config{25U, 0, hl::UnknownCellPolicy::exclude, 42U};

  const auto first = hl::sample_global_particles(grid, config);
  const auto second = hl::sample_global_particles(grid, config);

  ASSERT_EQ(first.size(), second.size());

  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_DOUBLE_EQ(first[index].pose.x, second[index].pose.x);
    EXPECT_DOUBLE_EQ(first[index].pose.y, second[index].pose.y);
    EXPECT_DOUBLE_EQ(first[index].pose.yaw, second[index].pose.yaw);
    EXPECT_DOUBLE_EQ(first[index].weight, second[index].weight);
  }
}

TEST(GlobalParticleSampling, RejectsMapWithoutEligibleCells)
{
  const std::array<std::int8_t, 2> cells{50, 100};
  const hl::OccupancyGridView grid{2U, 1U, 1.0, {}, cells};

  EXPECT_THROW(
    hl::sample_global_particles(
      grid,
      {10U, 0, hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);
}

TEST(GlobalParticleSampling, RejectsInvalidInput)
{
  const std::array<std::int8_t, 1> valid_cells{0};
  const std::array<std::int8_t, 1> invalid_cells{-2};

  EXPECT_THROW(
    hl::sample_global_particles(
      {1U, 1U, 1.0, {}, valid_cells},
      {0U, 0, hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);

  EXPECT_THROW(
    hl::sample_global_particles(
      {0U, 1U, 1.0, {}, valid_cells},
      {1U, 0, hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);

  EXPECT_THROW(
    hl::sample_global_particles(
      {1U, 1U, 0.0, {}, valid_cells},
      {1U, 0, hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);

  EXPECT_THROW(
    hl::sample_global_particles(
      {1U, 1U, 1.0, {NAN, 0.0, 0.0}, valid_cells},
      {1U, 0, hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);

  EXPECT_THROW(
    hl::sample_global_particles(
      {1U, 2U, 1.0, {}, valid_cells},
      {1U, 0, hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);

  EXPECT_THROW(
    hl::sample_global_particles(
      {1U, 1U, 1.0, {}, invalid_cells},
      {1U, 0, hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);

  EXPECT_THROW(
    hl::sample_global_particles(
      {1U, 1U, 1.0, {}, valid_cells},
      {1U, static_cast<std::int8_t>(-1), hl::UnknownCellPolicy::exclude, 1U}),
    std::invalid_argument);
}