#pragma once

namespace hybrid_localization
{

[[nodiscard]] double normalize_angle(double angle) noexcept;

[[nodiscard]] double angle_difference(
  double lhs,
  double rhs) noexcept;

}  // namespace hybrid_localization
