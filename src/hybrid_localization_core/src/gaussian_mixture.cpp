#include "hybrid_localization_core/gaussian_mixture.hpp"

#include "hybrid_localization_core/gaussian_statistics.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <numeric>

namespace hybrid_localization
{

GaussianMixture fit_gaussian_mixture(
  const std::span<const WeightedParticle> particles,
  const ParticleClusteringResult & clustering)
{
  /*
   * This validates the particles and gives us one authoritative normalized
   * weight for every original particle.
   */
  const auto normalized = normalize_weights(particles);

  std::vector<bool> retained(
    normalized.size(),
    false);

  GaussianMixture mixture;
  mixture.components.reserve(clustering.clusters.size());

  for (const auto & cluster : clustering.clusters) {
    if (cluster.particle_indices.empty()) {
      throw std::invalid_argument(
        "Particle cluster must not be empty");
    }

    for (const std::size_t index : cluster.particle_indices) {
      if (index >= normalized.size()) {
        throw std::invalid_argument(
          "Particle cluster index is out of range");
      }

      if (retained[index]) {
        throw std::invalid_argument(
          "Particle occurs in more than one retained cluster");
      }

      retained[index] = true;
    }

    mixture.components.push_back(
      fit_gaussian(
        normalized,
        cluster.particle_indices));
  }

  /*
   * Calculate discarded mass from every particle not represented by a
   * retained component. This includes both explicit noise particles and any
   * particle omitted from a manually constructed clustering result.
   */
  for (std::size_t index = 0;
       index < normalized.size();
       ++index)
  {
    if (!retained[index]) {
      mixture.discarded_weight +=
        normalized[index].weight;
    }
  }

  std::sort(
    mixture.components.begin(),
    mixture.components.end(),
    [](const GaussianComponent & lhs,
       const GaussianComponent & rhs)
    {
      return lhs.weight > rhs.weight;
    });

  const double represented_weight =
    std::accumulate(
      mixture.components.begin(),
      mixture.components.end(),
      0.0,
      [](const double total,
         const GaussianComponent & component)
      {
        return total + component.weight;
      });

  constexpr double mass_tolerance = 1e-12;

  if (std::abs(
        represented_weight +
        mixture.discarded_weight -
        1.0) > mass_tolerance)
  {
    throw std::runtime_error(
      "Gaussian mixture probability mass is inconsistent");
  }

  return mixture;
}

}  // namespace hybrid_localization