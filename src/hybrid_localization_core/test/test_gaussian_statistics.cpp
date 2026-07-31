#include <cmath>    // std::abs
#include <numbers>  // std::numbers::pi
#include <vector>   // std::vector

#include <gtest/gtest.h>

// Public interface of the library to test se2_statistics.
#include "hybrid_localization_core/gaussian_statistics.hpp"

namespace hl = hybrid_localization;

/**
 * Verify that repeated identical samples have zero spread.
 *
 * Particle weights differ, but both particles represent exactly the same
 * pose. Their weighted mean therefore equals that pose, and every residual
 * from the mean is zero.
 *
 * Consequently all nine entries of the 3x3 covariance matrix must be zero.
 */
TEST(Se2Statistics, IdenticalParticlesHaveZeroCovariance)
{
  const std::vector<hl::WeightedParticle> particles{
    {
      {1.0, 2.0, 0.5},
      0.4
    },
    {
      {1.0, 2.0, 0.5},
      0.6
    }
  };

  const auto gaussian = hl::fit_gaussian(particles);

  for (const double value : gaussian.covariance) {
    EXPECT_NEAR(value, 0.0, 1e-12);
  }
}
