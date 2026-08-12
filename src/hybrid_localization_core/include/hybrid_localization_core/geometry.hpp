#pragma once

namespace hybrid_localization
{

/// Normalize an angle in radians to the half-open interval [-pi, pi).
///
/// The function is noexcept and is used consistently for SE(2) yaw values
/// throughout the core library.
[[nodiscard]] double normalize_angle(double angle) noexcept;

/// Return the shortest signed wrapped angular difference lhs - rhs in radians.
///
/// The result lies in [-pi, pi). This avoids discontinuities when comparing
/// headings across the -pi/pi wrap boundary.
[[nodiscard]] double angle_difference(
  double lhs,
  double rhs) noexcept;

}  // namespace hybrid_localization
