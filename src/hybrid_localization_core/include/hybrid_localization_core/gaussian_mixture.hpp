#pragma once

#include <span>
#include <vector>

#include "hybrid_localization_core/particle_clustering.hpp"
#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// A Gaussian-mixture approximation of a particle belief.
///
/// Component weights retain their absolute normalized probability mass.
/// Therefore:
///
///   sum(component.weight) + discarded_weight == 1
///
/// within normal floating-point tolerance.
struct GaussianMixture
{
  std::vector<GaussianComponent> components{};

  /// Normalized particle probability mass not represented by a component.
  double discarded_weight{0.0};
};

/// Convert retained particle clusters into Gaussian components.
///
/// One Gaussian component is fitted to each retained cluster. Component
/// weights represent absolute normalized particle mass rather than being
/// renormalized over only the retained clusters.
///
/// Particles that are not referenced by a retained cluster contribute to
/// discarded_weight.
///
/// Throws std::invalid_argument when:
/// - the particle set is invalid;
/// - a cluster is empty;
/// - an index is out of range;
/// - an index occurs more than once across retained clusters.
[[nodiscard]] GaussianMixture fit_gaussian_mixture(
  std::span<const WeightedParticle> particles,
  const ParticleClusteringResult & clustering);

}  // namespace hybrid_localization