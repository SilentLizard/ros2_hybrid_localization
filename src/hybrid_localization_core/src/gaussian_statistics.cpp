#include "hybrid_localization_core/gaussian_statistics.hpp"

#include "hybrid_localization_core/geometry.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace hybrid_localization
{
namespace
{

void validate_indices(
  const std::span<const WeightedParticle> particles,
  const std::span<const std::size_t> indices)
{
  if (indices.empty()) {
    throw std::invalid_argument(
      "Gaussian fitting requires at least one particle index");
  }

  std::vector<bool> seen(particles.size(), false);

  for (const std::size_t index : indices) {
    if (index >= particles.size()) {
      throw std::invalid_argument(
        "Particle index is out of range");
    }

    if (seen[index]) {
      throw std::invalid_argument(
        "Particle index occurs more than once");
    }

    seen[index] = true;
  }
}

[[nodiscard]] GaussianComponent fit_normalized_gaussian(
  const std::span<const WeightedParticle> normalized_particles,
  const std::span<const std::size_t> indices)
{
  double component_weight = 0.0;
  double mean_x = 0.0;
  double mean_y = 0.0;
  double weighted_sine = 0.0;
  double weighted_cosine = 0.0;

  for (const std::size_t index : indices) {
    const auto & particle = normalized_particles[index];

    component_weight += particle.weight;
    mean_x += particle.weight * particle.pose.x;
    mean_y += particle.weight * particle.pose.y;
    weighted_sine +=
      particle.weight * std::sin(particle.pose.yaw);
    weighted_cosine +=
      particle.weight * std::cos(particle.pose.yaw);
  }

  if (!std::isfinite(component_weight) ||
      component_weight <= 0.0)
  {
    throw std::invalid_argument(
      "Selected particles must have positive total weight");
  }

  /*
   * Convert absolute particle weights into weights conditional on belonging
   * to this component.
   */
  mean_x /= component_weight;
  mean_y /= component_weight;
  weighted_sine /= component_weight;
  weighted_cosine /= component_weight;

  constexpr double minimum_resultant_length = 1e-12;

  if (std::hypot(weighted_sine, weighted_cosine) <
      minimum_resultant_length)
  {
    throw std::domain_error(
      "Circular mean is undefined for selected particles");
  }

  GaussianComponent component{};
  component.mean.x = mean_x;
  component.mean.y = mean_y;
  component.mean.yaw =
    std::atan2(weighted_sine, weighted_cosine);
  component.weight = component_weight;
  component.sample_count = indices.size();

  for (const std::size_t index : indices) {
    const auto & particle = normalized_particles[index];

    const double conditional_weight =
      particle.weight / component_weight;

    const std::array<double, 3> residual{
      particle.pose.x - component.mean.x,
      particle.pose.y - component.mean.y,
      angle_difference(
        particle.pose.yaw,
        component.mean.yaw)
    };

    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        component.covariance[row * 3 + column] +=
          conditional_weight *
          residual[row] *
          residual[column];
      }
    }
  }

  return component;
}

}  // namespace

GaussianComponent fit_gaussian(
  const std::span<const WeightedParticle> particles)
{
  const auto normalized = normalize_weights(particles);

  std::vector<std::size_t> indices(normalized.size());

  for (std::size_t index = 0; index < indices.size(); ++index) {
    indices[index] = index;
  }

  return fit_normalized_gaussian(
    normalized,
    indices);
}

GaussianComponent fit_gaussian(
  const std::span<const WeightedParticle> particles,
  const std::span<const std::size_t> indices)
{
  const auto normalized = normalize_weights(particles);

  validate_indices(normalized, indices);

  return fit_normalized_gaussian(
    normalized,
    indices);
}

}  // namespace hybrid_localization