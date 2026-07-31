#include <cmath>    // std::abs
#include <numbers>  // std::numbers::pi
#include <vector>   // std::vector

#include <gtest/gtest.h>

// Public interface of the library to test se2_statistics.
#include "hybrid_localization_core/geometry.hpp"

namespace hl = hybrid_localization;


/**
 * Verify the documented normalization interval [-pi, pi).
 *
 * The interval includes -pi but excludes +pi. Therefore both +3pi and -3pi
 * normalize to -pi.
 *
 * The test also verifies that complete rotations normalize to zero and that
 * angles already inside the requested interval remain unchanged.
 */
TEST(Se2Statistics, NormalizesAnglesToMinusPiInclusiveInterval)
{
  // 3pi represents the same orientation as pi. Since +pi is excluded from
  // [-pi, pi), the canonical result is -pi.
  EXPECT_NEAR(
    hl::normalize_angle(3.0 * std::numbers::pi),
    -std::numbers::pi,
    1e-12);

  // -3pi also represents the same orientation as -pi.
  EXPECT_NEAR(
    hl::normalize_angle(-3.0 * std::numbers::pi),
    -std::numbers::pi,
    1e-12);

  // One complete positive rotation should return to zero.
  EXPECT_NEAR(
    hl::normalize_angle(2.0 * std::numbers::pi),
    0.0,
    1e-12);

  // One complete negative rotation should also return to zero.
  EXPECT_NEAR(
    hl::normalize_angle(-2.0 * std::numbers::pi),
    0.0,
    1e-12);

  // An angle already inside the normalization interval should remain intact.
  EXPECT_NEAR(
    hl::normalize_angle(0.5 * std::numbers::pi),
    0.5 * std::numbers::pi,
    1e-12);
}