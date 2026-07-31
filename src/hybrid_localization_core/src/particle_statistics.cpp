#include "hybrid_localization_core/particle_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace hybrid_localization
{
namespace
{

void validate_particles(
  const std::span<const WeightedParticle> particles)
{
  if (particles.empty()) {
    throw std::invalid_argument(
      "Particle set must not be empty");
  }

  for (std::size_t index = 0; index < particles.size(); ++index) {
    const auto & particle = particles[index];

    if (!std::isfinite(particle.pose.x) ||
        !std::isfinite(particle.pose.y) ||
        !std::isfinite(particle.pose.yaw))
    {
      throw std::invalid_argument(
        "Particle pose values must be finite at index " +
        std::to_string(index));
    }

    if (!std::isfinite(particle.weight)) {
      throw std::invalid_argument(
        "Particle weight must be finite at index " +
        std::to_string(index));
    }

    if (particle.weight < 0.0) {
      throw std::invalid_argument(
        "Particle weight must be non-negative at index " +
        std::to_string(index));
    }
  }
}

}  // namespace

std::vector<WeightedParticle> normalize_weights(
  const std::span<const WeightedParticle> particles)
{
  validate_particles(particles);

  const double total_weight = std::accumulate(
    particles.begin(),
    particles.end(),
    0.0,
    [](const double sum, const WeightedParticle & particle) {
      return sum + particle.weight;
    });

  if (!std::isfinite(total_weight) || total_weight <= 0.0) {
    throw std::invalid_argument(
      "Particle weights must have a positive finite sum");
  }

  std::vector<WeightedParticle> normalized;
  normalized.reserve(particles.size());

  std::transform(
    particles.begin(),
    particles.end(),
    std::back_inserter(normalized),
    [total_weight](const WeightedParticle & particle) {
      auto result = particle;
      result.weight /= total_weight;
      return result;
    });

  return normalized;
}

Pose2d weighted_mean(
  const std::span<const WeightedParticle> particles)
{
  const auto normalized = normalize_weights(particles);

  Pose2d mean{};
  double weighted_sine = 0.0;
  double weighted_cosine = 0.0;

  for (const auto & particle : normalized) {
    mean.x += particle.weight * particle.pose.x;
    mean.y += particle.weight * particle.pose.y;

    weighted_sine +=
      particle.weight * std::sin(particle.pose.yaw);

    weighted_cosine +=
      particle.weight * std::cos(particle.pose.yaw);
  }

  /*
   * atan2(0, 0) is mathematically undefined. This can occur when the
   * orientation distribution is perfectly balanced, for example with equal
   * weights at 0 and pi.
   *
   * Returning zero would falsely imply a well-defined direction, so reject
   * the ambiguous mean instead.
   */
  constexpr double minimum_resultant_length = 1e-12;

  if (std::hypot(weighted_sine, weighted_cosine) <
      minimum_resultant_length)
  {
    throw std::domain_error(
      "Circular mean is undefined for this orientation distribution");
  }

  mean.yaw = std::atan2(weighted_sine, weighted_cosine);
  return mean;
}

double effective_sample_size(
  const std::span<const WeightedParticle> particles)
{
  const auto normalized = normalize_weights(particles);

  const double squared_weight_sum = std::accumulate(
    normalized.begin(),
    normalized.end(),
    0.0,
    [](const double sum, const WeightedParticle & particle) {
      return sum + particle.weight * particle.weight;
    });

  return 1.0 / squared_weight_sum;
}

}  // namespace hybrid_localization