#pragma once

#include <array>
#include <cstddef>

namespace hybrid_localization
{

/// A planar pose in SE(2).
struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A weighted pose sample in a particle-based belief representation.
struct WeightedParticle
{
  Pose2d pose{};
  double weight{0.0};
};

/// A locally Gaussian pose hypothesis.
///
/// Covariance is stored row-major for [x, y, yaw].
struct GaussianComponent
{
  Pose2d mean{};
  std::array<double, 9> covariance{};
  double weight{0.0};
  std::size_t sample_count{0};
};

}  // namespace hybrid_localization
