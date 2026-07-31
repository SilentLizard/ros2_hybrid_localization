#include "hybrid_localization_core/geometry.hpp"

#include <cmath>
#include <numbers>

namespace hybrid_localization
{

double normalize_angle(const double angle) noexcept
{
  constexpr double two_pi = 2.0 * std::numbers::pi;

  double wrapped = std::fmod(
    angle + std::numbers::pi,
    two_pi);

  if (wrapped < 0.0) {
    wrapped += two_pi;
  }

  return wrapped - std::numbers::pi;
}

double angle_difference(
  const double lhs,
  const double rhs) noexcept
{
  return normalize_angle(lhs - rhs);
}

}  // namespace hybrid_localization
