#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/global_particle_sampling.hpp"
#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Normalized evidence used to increase global exploration during recovery.
struct RecoveryAllocationSignals
{
  /// Recovery failure severity in [0, 1].
  double failure_score{0.0};
};

/// Configuration for adaptive local/global recovery sampling.
struct AdaptiveRecoverySamplingConfig
{
  /// Exact total number of particles returned.
  std::size_t particle_count{0U};

  /// Baseline fraction assigned to global map exploration.
  double base_global_fraction{0.05};

  /// Lower and upper bounds for the computed global fraction.
  double minimum_global_fraction{0.0};
  double maximum_global_fraction{1.0};

  /// Linear gains applied to normalized recovery evidence.
  double discarded_weight_gain{0.0};
  double mixture_entropy_gain{0.0};
  double failure_score_gain{0.0};

  /// Local Gaussian sampling controls.
  double covariance_inflation_factor{1.0};
  std::size_t minimum_samples_per_component{0U};

  /// Global occupancy-grid sampling controls.
  std::int8_t maximum_occupancy{0};
  UnknownCellPolicy unknown_cell_policy{UnknownCellPolicy::exclude};

  /// Base seed from which deterministic child seeds are derived.
  std::uint64_t random_seed{0U};
};

/// Result of adaptive recovery sampling, including the allocation decision.
struct AdaptiveRecoverySamplingResult
{
  std::vector<WeightedParticle> particles{};
  std::size_t local_particle_count{0U};
  std::size_t global_particle_count{0U};
  double global_fraction{0.0};
  double normalized_mixture_entropy{0.0};
};

/// Compute normalized Shannon entropy over represented component mass.
///
/// Returns zero for zero or one retained component. Component weights are
/// renormalized over represented mass; discarded_weight is not a component.
[[nodiscard]] double normalized_mixture_entropy(
  const GaussianMixture & mixture);

/// Generate one recovery belief by combining local GMM and global map samples.
///
/// The global fraction is computed as a bounded linear policy using discarded
/// mass, normalized mixture entropy, and recovery failure severity. Exact local
/// and global particle counts are derived from the requested total. The final
/// combined belief is shuffled deterministically and assigned equal weights.
[[nodiscard]] AdaptiveRecoverySamplingResult sample_adaptive_recovery_particles(
  const GaussianMixture & mixture,
  const OccupancyGridView & grid,
  const RecoveryAllocationSignals & signals,
  const AdaptiveRecoverySamplingConfig & config);

}  // namespace hybrid_localization