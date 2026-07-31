#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_localization_core/gaussian_mixture.hpp"

namespace hl = hybrid_localization;

TEST(GaussianMixture, ConvertsTwoClustersIntoTwoComponents)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 3.0},
    {{2.0, 0.0, 0.0}, 3.0},

    {{10.0, 0.0, 1.0}, 2.0},
    {{12.0, 0.0, 1.0}, 2.0}
  };

  hl::ParticleClusteringResult clustering;

  clustering.clusters.push_back(
    {{0U, 1U}, 0.6});

  clustering.clusters.push_back(
    {{2U, 3U}, 0.4});

  const auto mixture =
    hl::fit_gaussian_mixture(
      particles,
      clustering);

  ASSERT_EQ(mixture.components.size(), 2U);

  EXPECT_NEAR(
    mixture.components[0].weight,
    0.6,
    1e-12);

  EXPECT_NEAR(
    mixture.components[0].mean.x,
    1.0,
    1e-12);

  EXPECT_NEAR(
    mixture.components[0].covariance[0],
    1.0,
    1e-12);

  EXPECT_NEAR(
    mixture.components[1].weight,
    0.4,
    1e-12);

  EXPECT_NEAR(
    mixture.components[1].mean.x,
    11.0,
    1e-12);

  EXPECT_NEAR(
    mixture.discarded_weight,
    0.0,
    1e-12);
}

TEST(GaussianMixture, ReportsDiscardedParticleMass)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 4.0},
    {{2.0, 0.0, 0.0}, 4.0},
    {{100.0, 100.0, 0.0}, 2.0}
  };

  hl::ParticleClusteringResult clustering;
  clustering.clusters.push_back(
    {{0U, 1U}, 0.8});

  clustering.noise_indices.push_back(2U);

  const auto mixture =
    hl::fit_gaussian_mixture(
      particles,
      clustering);

  ASSERT_EQ(mixture.components.size(), 1U);

  EXPECT_NEAR(
    mixture.components.front().weight,
    0.8,
    1e-12);

  EXPECT_NEAR(
    mixture.discarded_weight,
    0.2,
    1e-12);

  EXPECT_NEAR(
    mixture.components.front().weight +
      mixture.discarded_weight,
    1.0,
    1e-12);
}

TEST(GaussianMixture, SortsComponentsByDescendingWeight)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0},
    {{1.0, 0.0, 0.0}, 1.0},

    {{10.0, 0.0, 0.0}, 4.0},
    {{11.0, 0.0, 0.0}, 4.0}
  };

  hl::ParticleClusteringResult clustering;

  /*
   * Deliberately insert the smaller cluster first.
   */
  clustering.clusters.push_back(
    {{0U, 1U}, 0.2});

  clustering.clusters.push_back(
    {{2U, 3U}, 0.8});

  const auto mixture =
    hl::fit_gaussian_mixture(
      particles,
      clustering);

  ASSERT_EQ(mixture.components.size(), 2U);

  EXPECT_NEAR(
    mixture.components[0].weight,
    0.8,
    1e-12);

  EXPECT_NEAR(
    mixture.components[1].weight,
    0.2,
    1e-12);
}

TEST(GaussianMixture, FitsWrappedYawWithinCluster)
{
  constexpr double degrees_to_radians =
    std::numbers::pi / 180.0;

  const std::vector<hl::WeightedParticle> particles{
    {
      {0.0, 0.0, 179.0 * degrees_to_radians},
      1.0
    },
    {
      {0.0, 0.0, -179.0 * degrees_to_radians},
      1.0
    }
  };

  hl::ParticleClusteringResult clustering;
  clustering.clusters.push_back(
    {{0U, 1U}, 1.0});

  const auto mixture =
    hl::fit_gaussian_mixture(
      particles,
      clustering);

  ASSERT_EQ(mixture.components.size(), 1U);

  EXPECT_NEAR(
    std::abs(mixture.components[0].mean.yaw),
    std::numbers::pi,
    1e-12);

  const double expected_variance =
    degrees_to_radians * degrees_to_radians;

  EXPECT_NEAR(
    mixture.components[0].covariance[8],
    expected_variance,
    1e-12);
}

TEST(GaussianMixture, RejectsOutOfRangeClusterIndex)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0}
  };

  hl::ParticleClusteringResult clustering;
  clustering.clusters.push_back(
    {{1U}, 1.0});

  EXPECT_THROW(
    static_cast<void>(
      hl::fit_gaussian_mixture(
        particles,
        clustering)),
    std::invalid_argument);
}

TEST(GaussianMixture, RejectsParticleInMultipleClusters)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.5},
    {{1.0, 0.0, 0.0}, 0.5}
  };

  hl::ParticleClusteringResult clustering;

  clustering.clusters.push_back(
    {{0U}, 0.5});

  clustering.clusters.push_back(
    {{0U, 1U}, 1.0});

  EXPECT_THROW(
    static_cast<void>(
      hl::fit_gaussian_mixture(
        particles,
        clustering)),
    std::invalid_argument);
}

TEST(GaussianMixture, RejectsEmptyCluster)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0}
  };

  hl::ParticleClusteringResult clustering;
  clustering.clusters.push_back(
    {{}, 0.0});

  EXPECT_THROW(
    static_cast<void>(
      hl::fit_gaussian_mixture(
        particles,
        clustering)),
    std::invalid_argument);
}

TEST(GaussianMixture, AllowsAllParticlesToBeDiscarded)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0},
    {{10.0, 10.0, 1.0}, 1.0}
  };

  hl::ParticleClusteringResult clustering;
  clustering.noise_indices = {0U, 1U};

  const auto mixture =
    hl::fit_gaussian_mixture(
      particles,
      clustering);

  EXPECT_TRUE(mixture.components.empty());

  EXPECT_NEAR(
    mixture.discarded_weight,
    1.0,
    1e-12);
}