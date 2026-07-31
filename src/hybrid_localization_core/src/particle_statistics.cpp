#include "hybrid_localization_core/particle_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>

namespace hybrid_localization
{

std::vector<WeightedParticle> normalize_weights(
  const std::span<const WeightedParticle> particles)
{
  if (particles.empty()) {
    throw std::invalid_argument("Cannot normalize an empty particle set");
  }

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
  double sin_sum = 0.0;
  double cos_sum = 0.0;

  for (const auto & particle : normalized) {
    mean.x += particle.weight * particle.pose.x;
    mean.y += particle.weight * particle.pose.y;

    sin_sum += particle.weight * std::sin(particle.pose.yaw);
    cos_sum += particle.weight * std::cos(particle.pose.yaw);
  }

  mean.yaw = std::atan2(sin_sum, cos_sum);
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
