#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_localization_core/particle_statistics.hpp"

namespace hl = hybrid_localization;

TEST(ParticleStatistics, NormalizesWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0},
    {{1.0, 0.0, 0.0}, 2.0},
    {{2.0, 0.0, 0.0}, 3.0}
  };

  const auto normalized = hl::normalize_weights(particles);

  ASSERT_EQ(normalized.size(), 3U);

  EXPECT_NEAR(normalized[0].weight, 1.0 / 6.0, 1e-12);
  EXPECT_NEAR(normalized[1].weight, 2.0 / 6.0, 1e-12);
  EXPECT_NEAR(normalized[2].weight, 3.0 / 6.0, 1e-12);

  const double total =
    normalized[0].weight +
    normalized[1].weight +
    normalized[2].weight;

  EXPECT_NEAR(total, 1.0, 1e-12);
}

TEST(ParticleStatistics, DoesNotModifyOriginalWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 2.0},
    {{1.0, 0.0, 0.0}, 3.0}
  };

  const auto normalized = hl::normalize_weights(particles);

  EXPECT_DOUBLE_EQ(particles[0].weight, 2.0);
  EXPECT_DOUBLE_EQ(particles[1].weight, 3.0);

  EXPECT_NEAR(normalized[0].weight, 0.4, 1e-12);
  EXPECT_NEAR(normalized[1].weight, 0.6, 1e-12);
}

TEST(ParticleStatistics, ComputesWeightedPositionMean)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0},
    {{10.0, 4.0, 0.0}, 3.0}
  };

  const auto mean = hl::weighted_mean(particles);

  EXPECT_NEAR(mean.x, 7.5, 1e-12);
  EXPECT_NEAR(mean.y, 3.0, 1e-12);
  EXPECT_NEAR(mean.yaw, 0.0, 1e-12);
}

TEST(ParticleStatistics, ComputesWeightedCircularYawMean)
{
  constexpr double degrees_to_radians =
    std::numbers::pi / 180.0;

  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 10.0 * degrees_to_radians}, 1.0},
    {{0.0, 0.0, 30.0 * degrees_to_radians}, 3.0}
  };

  const auto mean = hl::weighted_mean(particles);

  /*
   * Circular means are not generally identical to arithmetic means, so use
   * the independently calculated atan2 result.
   */
  const double expected = std::atan2(
    std::sin(10.0 * degrees_to_radians) +
      3.0 * std::sin(30.0 * degrees_to_radians),
    std::cos(10.0 * degrees_to_radians) +
      3.0 * std::cos(30.0 * degrees_to_radians));

  EXPECT_NEAR(mean.yaw, expected, 1e-12);
}

TEST(ParticleStatistics, HandlesCircularMeanAcrossAngleWraparound)
{
  constexpr double degrees_to_radians =
    std::numbers::pi / 180.0;

  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 179.0 * degrees_to_radians}, 0.5},
    {{0.0, 0.0, -179.0 * degrees_to_radians}, 0.5}
  };

  const auto mean = hl::weighted_mean(particles);

  EXPECT_NEAR(
    std::abs(mean.yaw),
    std::numbers::pi,
    1e-12);
}

TEST(ParticleStatistics, CalculatesEffectiveSampleSizeForEqualWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0},
    {{1.0, 0.0, 0.0}, 1.0},
    {{2.0, 0.0, 0.0}, 1.0},
    {{3.0, 0.0, 0.0}, 1.0}
  };

  EXPECT_NEAR(
    hl::effective_sample_size(particles),
    4.0,
    1e-12);
}

TEST(ParticleStatistics, CalculatesEffectiveSampleSizeForUnequalWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.7},
    {{1.0, 0.0, 0.0}, 0.1},
    {{2.0, 0.0, 0.0}, 0.1},
    {{3.0, 0.0, 0.0}, 0.1}
  };

  const double expected =
    1.0 / (0.7 * 0.7 + 3.0 * 0.1 * 0.1);

  EXPECT_NEAR(
    hl::effective_sample_size(particles),
    expected,
    1e-12);
}

TEST(ParticleStatistics, RejectsEmptyParticleSet)
{
  const std::vector<hl::WeightedParticle> particles;

  EXPECT_THROW(
    static_cast<void>(hl::normalize_weights(particles)),
    std::invalid_argument);
}

TEST(ParticleStatistics, RejectsAllZeroWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.0},
    {{1.0, 0.0, 0.0}, 0.0}
  };

  EXPECT_THROW(
    static_cast<void>(hl::normalize_weights(particles)),
    std::invalid_argument);
}

TEST(ParticleStatistics, RejectsNegativeWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, -1.0},
    {{1.0, 0.0, 0.0}, 2.0}
  };

  EXPECT_THROW(
    static_cast<void>(hl::normalize_weights(particles)),
    std::invalid_argument);
}

TEST(ParticleStatistics, RejectsNonFiniteWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {
      {0.0, 0.0, 0.0},
      std::numeric_limits<double>::quiet_NaN()
    }
  };

  EXPECT_THROW(
    static_cast<void>(hl::normalize_weights(particles)),
    std::invalid_argument);
}

TEST(ParticleStatistics, RejectsNonFinitePoseValues)
{
  const std::vector<hl::WeightedParticle> particles{
    {
      {
        std::numeric_limits<double>::infinity(),
        0.0,
        0.0
      },
      1.0
    }
  };

  EXPECT_THROW(
    static_cast<void>(hl::weighted_mean(particles)),
    std::invalid_argument);
}

TEST(ParticleStatistics, RejectsUndefinedCircularMean)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.5},
    {{0.0, 0.0, std::numbers::pi}, 0.5}
  };

  EXPECT_THROW(
    static_cast<void>(hl::weighted_mean(particles)),
    std::domain_error);
}