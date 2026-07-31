#include <cmath>    // std::abs
#include <numbers>  // std::numbers::pi
#include <vector>   // std::vector

#include <gtest/gtest.h>

// Public interface of the library to test se2_statistics.
#include "hybrid_localization_core/particle_statistics.hpp"

namespace hl = hybrid_localization;


/**
 * Verify that orientation averaging handles the wraparound at +/-pi.
 *
 * The orientations +179 degrees and -179 degrees are only two degrees apart.
 * An ordinary arithmetic mean would incorrectly produce zero degrees.
 *
 * A circular mean should produce an orientation close to either +pi or -pi.
 * Since both values represent the same physical direction, the test compares
 * the absolute value of the resulting angle with pi.
 */
TEST(ParticleStatistics, CircularMeanHandlesAngleWraparound)
{
  const std::vector<hl::WeightedParticle> particles{
    {
      {
        0.0,
        0.0,
        179.0 * std::numbers::pi / 180.0
      },
      0.5
    },
    {
      {
        0.0,
        0.0,
        -179.0 * std::numbers::pi / 180.0
      },
      0.5
    }
  };

  const auto mean = hl::weighted_mean(particles);

  // The result may be represented as either +pi or -pi. Both describe the
  // same physical orientation, so compare its absolute value with pi.
  EXPECT_NEAR(
    std::abs(mean.yaw),
    std::numbers::pi,
    1e-6);
}


/**
 * Verify weighted averaging of position.
 *
 * The second particle has three times the weight of the first particle:
 *
 *   mean x = (1 * 0 + 3 * 10) / 4 = 7.5
 *   mean y = (1 * 0 + 3 *  4) / 4 = 3.0
 *
 * Both yaw values are zero, so the circular yaw mean must also be zero.
 */
TEST(Se2Statistics, ComputesWeightedMean)
{
  const std::vector<hl::WeightedParticle> particles{
    {
      {0.0, 0.0, 0.0},
      1.0
    },
    {
      {10.0, 4.0, 0.0},
      3.0
    }
  };

  const auto mean = hl::weighted_mean(particles);

  EXPECT_NEAR(mean.x, 7.5, 1e-12);
  EXPECT_NEAR(mean.y, 3.0, 1e-12);
  EXPECT_NEAR(mean.yaw, 0.0, 1e-12);
}


/**
 * Verify the effective sample size for equally weighted particles.
 *
 * After normalization, each of four equal particles has weight 0.25:
 *
 *   sum(w_i^2) = 4 * 0.25^2 = 0.25
 *   N_eff      = 1 / 0.25     = 4
 *
 * This means all four particles contribute equally to the estimate.
 */
TEST(Se2Statistics, EffectiveSampleSizeReflectsWeights)
{
  const std::vector<hl::WeightedParticle> equal_particles{
    {
      {0.0, 0.0, 0.0},
      1.0
    },
    {
      {1.0, 0.0, 0.0},
      1.0
    },
    {
      {2.0, 0.0, 0.0},
      1.0
    },
    {
      {3.0, 0.0, 0.0},
      1.0
    }
  };

  EXPECT_NEAR(
    hl::effective_sample_size(equal_particles),
    4.0,
    1e-12);
}