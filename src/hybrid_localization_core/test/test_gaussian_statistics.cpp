#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_localization_core/gaussian_statistics.hpp"

namespace hl = hybrid_localization;

TEST(GaussianStatistics, IdenticalParticlesHaveZeroCovariance)
{
  const std::vector<hl::WeightedParticle> particles{
    {{1.0, 2.0, 0.5}, 0.4},
    {{1.0, 2.0, 0.5}, 0.6}
  };

  const auto gaussian = hl::fit_gaussian(particles);

  EXPECT_NEAR(gaussian.mean.x, 1.0, 1e-12);
  EXPECT_NEAR(gaussian.mean.y, 2.0, 1e-12);
  EXPECT_NEAR(gaussian.mean.yaw, 0.5, 1e-12);

  for (const double value : gaussian.covariance) {
    EXPECT_NEAR(value, 0.0, 1e-12);
  }
}

TEST(GaussianStatistics, ComputesPositionCovariance)
{
  /*
   * Two equally weighted samples:
   *
   *   p0 = (0, 0)
   *   p1 = (2, 4)
   *
   * Mean:
   *
   *   mu = (1, 2)
   *
   * Residuals:
   *
   *   r0 = (-1, -2)
   *   r1 = ( 1,  2)
   *
   * Weighted covariance:
   *
   *   var(x) = 1
   *   var(y) = 4
   *   cov(x,y) = 2
   */
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.5},
    {{2.0, 4.0, 0.0}, 0.5}
  };

  const auto gaussian = hl::fit_gaussian(particles);

  EXPECT_NEAR(gaussian.mean.x, 1.0, 1e-12);
  EXPECT_NEAR(gaussian.mean.y, 2.0, 1e-12);

  EXPECT_NEAR(gaussian.covariance[0], 1.0, 1e-12);
  EXPECT_NEAR(gaussian.covariance[1], 2.0, 1e-12);
  EXPECT_NEAR(gaussian.covariance[3], 2.0, 1e-12);
  EXPECT_NEAR(gaussian.covariance[4], 4.0, 1e-12);

  EXPECT_NEAR(gaussian.covariance[2], 0.0, 1e-12);
  EXPECT_NEAR(gaussian.covariance[5], 0.0, 1e-12);
  EXPECT_NEAR(gaussian.covariance[6], 0.0, 1e-12);
  EXPECT_NEAR(gaussian.covariance[7], 0.0, 1e-12);
  EXPECT_NEAR(gaussian.covariance[8], 0.0, 1e-12);
}

TEST(GaussianStatistics, ComputesYawCovarianceAcrossWraparound)
{
  constexpr double degrees_to_radians =
    std::numbers::pi / 180.0;

  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 179.0 * degrees_to_radians}, 0.5},
    {{0.0, 0.0, -179.0 * degrees_to_radians}, 0.5}
  };

  const auto gaussian = hl::fit_gaussian(particles);

  const double one_degree = degrees_to_radians;
  const double expected_yaw_variance =
    one_degree * one_degree;

  EXPECT_NEAR(
    gaussian.covariance[8],
    expected_yaw_variance,
    1e-12);

  EXPECT_LT(
    gaussian.covariance[8],
    0.001);
}

TEST(GaussianStatistics, ProducesSymmetricCovarianceMatrix)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 1.0},
    {{1.0, 2.0, 0.1}, 2.0},
    {{3.0, 1.0, 0.2}, 1.0}
  };

  const auto gaussian = hl::fit_gaussian(particles);

  EXPECT_NEAR(
    gaussian.covariance[1],
    gaussian.covariance[3],
    1e-12);

  EXPECT_NEAR(
    gaussian.covariance[2],
    gaussian.covariance[6],
    1e-12);

  EXPECT_NEAR(
    gaussian.covariance[5],
    gaussian.covariance[7],
    1e-12);
}