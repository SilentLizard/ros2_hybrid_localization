#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Configuration for local recovery sampling from a Gaussian mixture.
struct RecoverySamplingConfig
{
  /// Exact number of particles returned by the sampler.
  std::size_t particle_count{0U};

  /// Multiplier applied to every component covariance before sampling.
  double covariance_inflation_factor{1.0};

  /// Seed used by the deterministic pseudo-random number generator.
  std::uint64_t random_seed{0U};

  /// Optional lower bound for samples allocated to every component.
  std::size_t minimum_samples_per_component{0U};
};

/// Sample a local recovery particle belief from retained GMM components.
///
/// Component weights are renormalized over represented mixture mass for sample
/// allocation. discarded_weight is deliberately not sampled because its
/// original spatial distribution is no longer available.
///
/// Returned particles have equal normalized weights. Yaw is normalized to
/// [-pi, pi).
[[nodiscard]] std::vector<WeightedParticle> sample_recovery_particles(
  const GaussianMixture & mixture,
  const RecoverySamplingConfig & config);

}  // namespace hybrid_localization