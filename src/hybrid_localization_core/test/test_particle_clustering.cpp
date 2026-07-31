#include <algorithm>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>
#include <limits>

#include <gtest/gtest.h>

#include "hybrid_localization_core/particle_clustering.hpp"

namespace hl = hybrid_localization;

namespace
{

[[nodiscard]] hl::ParticleClusteringConfig test_config()
{
  hl::ParticleClusteringConfig config;
  config.position_scale = 0.25;
  config.yaw_scale = 0.25;
  config.epsilon = 1.0;
  config.minimum_neighbors = 2;
  config.minimum_core_weight = 0.0;
  config.minimum_cluster_weight = 0.0;
  return config;
}

}  // namespace

TEST(ParticleClustering, FindsTwoSeparatedClusters)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.00, 0.00, 0.00}, 1.0},
    {{0.05, 0.00, 0.02}, 1.0},
    {{0.00, 0.05, -0.02}, 1.0},

    {{5.00, 5.00, 1.00}, 1.0},
    {{5.05, 5.00, 1.02}, 1.0},
    {{5.00, 5.05, 0.98}, 1.0}
  };

  const auto result =
    hl::cluster_particles(particles, test_config());

  ASSERT_EQ(result.clusters.size(), 2U);
  EXPECT_TRUE(result.noise_indices.empty());

  EXPECT_EQ(result.clusters[0].particle_indices.size(), 3U);
  EXPECT_EQ(result.clusters[1].particle_indices.size(), 3U);

  EXPECT_NEAR(result.clusters[0].weight, 0.5, 1e-12);
  EXPECT_NEAR(result.clusters[1].weight, 0.5, 1e-12);
}

TEST(ParticleClustering, ClustersYawAcrossWraparound)
{
  constexpr double degrees_to_radians =
    std::numbers::pi / 180.0;

  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 179.0 * degrees_to_radians}, 1.0},
    {{0.0, 0.0, -179.0 * degrees_to_radians}, 1.0},
    {{0.0, 0.0, 178.0 * degrees_to_radians}, 1.0}
  };

  auto config = test_config();
  config.position_scale = 0.1;
  config.yaw_scale = 10.0 * degrees_to_radians;

  const auto result =
    hl::cluster_particles(particles, config);

  ASSERT_EQ(result.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.front().particle_indices.size(), 3U);
  EXPECT_TRUE(result.noise_indices.empty());
}

TEST(ParticleClustering, MarksIsolatedParticleAsNoise)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.00, 0.00, 0.00}, 1.0},
    {{0.05, 0.00, 0.00}, 1.0},
    {{0.00, 0.05, 0.00}, 1.0},

    {{10.0, 10.0, 0.00}, 1.0}
  };

  auto config = test_config();
  config.minimum_neighbors = 2;

  const auto result =
    hl::cluster_particles(particles, config);

  ASSERT_EQ(result.clusters.size(), 1U);
  ASSERT_EQ(result.noise_indices.size(), 1U);
  EXPECT_EQ(result.noise_indices.front(), 3U);
}

TEST(ParticleClustering, PreservesNormalizedClusterWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.00, 0.00, 0.00}, 4.0},
    {{0.05, 0.00, 0.00}, 2.0},

    {{5.00, 5.00, 0.00}, 1.0},
    {{5.05, 5.00, 0.00}, 1.0}
  };

  const auto result =
    hl::cluster_particles(particles, test_config());

  ASSERT_EQ(result.clusters.size(), 2U);

  EXPECT_NEAR(result.clusters[0].weight, 0.75, 1e-12);
  EXPECT_NEAR(result.clusters[1].weight, 0.25, 1e-12);
}

TEST(ParticleClustering, SortsClustersByDescendingWeight)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.00, 0.00, 0.00}, 1.0},
    {{0.05, 0.00, 0.00}, 1.0},

    {{5.00, 5.00, 0.00}, 4.0},
    {{5.05, 5.00, 0.00}, 4.0}
  };

  const auto result =
    hl::cluster_particles(particles, test_config());

  ASSERT_EQ(result.clusters.size(), 2U);
  EXPECT_GT(result.clusters[0].weight, result.clusters[1].weight);
}

TEST(ParticleClustering, RejectsClusterBelowMinimumWeight)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.00, 0.00, 0.00}, 9.0},
    {{0.05, 0.00, 0.00}, 9.0},

    {{5.00, 5.00, 0.00}, 1.0},
    {{5.05, 5.00, 0.00}, 1.0}
  };

  auto config = test_config();
  config.minimum_cluster_weight = 0.2;

  const auto result =
    hl::cluster_particles(particles, config);

  ASSERT_EQ(result.clusters.size(), 1U);

  /*
   * The second spatial group contains 10% of the normalized weight and is
   * therefore returned as noise after final cluster filtering.
   */
  ASSERT_EQ(result.noise_indices.size(), 2U);
  EXPECT_NEAR(result.clusters.front().weight, 0.9, 1e-12);
}

TEST(ParticleClustering, UsesMinimumCoreWeight)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.00, 0.00, 0.00}, 0.01},
    {{0.02, 0.00, 0.00}, 0.01},
    {{0.00, 0.02, 0.00}, 0.01},

    {{5.00, 5.00, 0.00}, 0.49},
    {{5.02, 5.00, 0.00}, 0.48}
  };

  auto config = test_config();
  config.minimum_neighbors = 2;
  config.minimum_core_weight = 0.1;

  const auto result =
    hl::cluster_particles(particles, config);

  ASSERT_EQ(result.clusters.size(), 1U);
  EXPECT_EQ(result.clusters.front().particle_indices.size(), 2U);
}

TEST(ParticleClustering, ComputesWrappedSE2Distance)
{
  constexpr double degrees_to_radians =
    std::numbers::pi / 180.0;

  hl::ParticleClusteringConfig config;
  config.position_scale = 1.0;
  config.yaw_scale = degrees_to_radians;
  config.epsilon = 10.0;

  const hl::Pose2d lhs{
    0.0,
    0.0,
    179.0 * degrees_to_radians
  };

  const hl::Pose2d rhs{
    0.0,
    0.0,
    -179.0 * degrees_to_radians
  };

  /*
   * The wrapped yaw difference is two degrees. With yaw_scale = one degree,
   * the normalized squared distance is 2² = 4.
   */
  EXPECT_NEAR(
    hl::particle_distance_squared(lhs, rhs, config),
    4.0,
    1e-12);
}

TEST(ParticleClustering, RejectsInvalidConfiguration)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0}
  };

  auto config = test_config();
  config.position_scale = 0.0;

  EXPECT_THROW(
    static_cast<void>(
      hl::cluster_particles(particles, config)),
    std::invalid_argument);
}

TEST(ParticleClusteringConfig, AcceptsDefaultConfiguration)
{
  const hl::ParticleClusteringConfig config;

  EXPECT_NO_THROW(
    hl::validate_particle_clustering_config(config));
}

TEST(ParticleClusteringConfig, RejectsNonPositivePositionScale)
{
  auto config = test_config();
  config.position_scale = 0.0;

  EXPECT_THROW(
    hl::validate_particle_clustering_config(config),
    std::invalid_argument);
}

TEST(ParticleClusteringConfig, RejectsNonPositiveYawScale)
{
  auto config = test_config();
  config.yaw_scale = 0.0;

  EXPECT_THROW(
    hl::validate_particle_clustering_config(config),
    std::invalid_argument);
}

TEST(ParticleClusteringConfig, RejectsNonPositiveEpsilon)
{
  auto config = test_config();
  config.epsilon = 0.0;

  EXPECT_THROW(
    hl::validate_particle_clustering_config(config),
    std::invalid_argument);
}

TEST(ParticleClusteringConfig, RejectsZeroMinimumNeighbors)
{
  auto config = test_config();
  config.minimum_neighbors = 0U;

  EXPECT_THROW(
    hl::validate_particle_clustering_config(config),
    std::invalid_argument);
}

TEST(ParticleClusteringConfig, RejectsInvalidWeightThresholds)
{
  auto config = test_config();
  config.minimum_core_weight = 1.1;

  EXPECT_THROW(
    hl::validate_particle_clustering_config(config),
    std::invalid_argument);

  config = test_config();
  config.minimum_cluster_weight = -0.1;

  EXPECT_THROW(
    hl::validate_particle_clustering_config(config),
    std::invalid_argument);
}

TEST(ParticleClusteringConfig, RejectsNonFiniteValues)
{
  auto config = test_config();
  config.epsilon =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(
    hl::validate_particle_clustering_config(config),
    std::invalid_argument);
}