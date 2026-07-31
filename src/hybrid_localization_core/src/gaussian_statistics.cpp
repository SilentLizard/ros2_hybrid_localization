#include "hybrid_localization_core/gaussian_statistics.hpp"

#include "hybrid_localization_core/geometry.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

#include <array>
#include <cstddef>

namespace hybrid_localization
{

GaussianComponent fit_gaussian(
  const std::span<const WeightedParticle> particles)
{
  const auto normalized = normalize_weights(particles);
  const Pose2d mean = weighted_mean(normalized);

  GaussianComponent component{};
  component.mean = mean;
  component.weight = 1.0;
  component.sample_count = normalized.size();

  for (const auto & particle : normalized) {
    const std::array<double, 3> residual{
      particle.pose.x - mean.x,
      particle.pose.y - mean.y,
      angle_difference(particle.pose.yaw, mean.yaw)
    };

    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        component.covariance[row * 3 + column] +=
          particle.weight *
          residual[row] *
          residual[column];
      }
    }
  }

  return component;
}

}  // namespace hybrid_localization