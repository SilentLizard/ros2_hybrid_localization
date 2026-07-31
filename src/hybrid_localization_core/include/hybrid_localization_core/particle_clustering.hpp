#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Parameters controlling weighted DBSCAN clustering in SE(2).
struct ParticleClusteringConfig
{
  /// Position distance represented by one normalized distance unit [m].
  double position_scale{0.25};

  /// Angular distance represented by one normalized distance unit [rad].
  double yaw_scale{0.35};

  /// Maximum normalized neighborhood distance.
  double epsilon{1.0};

  /// Minimum number of particles in a core neighborhood,
  /// including the center particle.
  std::size_t minimum_neighbors{5};

  /// Minimum normalized weight in a core particle's neighborhood.
  double minimum_core_weight{0.0};

  /// Minimum normalized total weight retained as a final cluster.
  double minimum_cluster_weight{0.01};
};

/// One detected particle mode.
///
/// Indices refer to the particle order supplied to cluster_particles().
struct ParticleCluster
{
  std::vector<std::size_t> particle_indices;

  /// Sum of normalized particle weights in this cluster.
  double weight{0.0};
};

/// Complete result of one clustering operation.
struct ParticleClusteringResult
{
  /// Clusters sorted by descending normalized weight.
  std::vector<ParticleCluster> clusters;

  /// Particles that did not belong to a retained cluster.
  std::vector<std::size_t> noise_indices;
};

/// Calculate the dimensionless squared SE(2) clustering distance.
///
/// Translation and yaw are independently scaled. Yaw uses the shortest
/// wrapped angular difference.
[[nodiscard]] double particle_distance_squared(
  const Pose2d & lhs,
  const Pose2d & rhs,
  const ParticleClusteringConfig & config);

/// Cluster a weighted particle population using weighted DBSCAN in SE(2).
///
/// Particle weights are normalized internally. Returned indices refer to the
/// original input order.
///
/// Throws std::invalid_argument when the input or configuration is invalid.
[[nodiscard]] ParticleClusteringResult cluster_particles(
  std::span<const WeightedParticle> particles,
  const ParticleClusteringConfig & config = {});

/// Validate particle-clustering configuration.
///
/// Throws std::invalid_argument if any configuration value is invalid.
void validate_particle_clustering_config(
  const ParticleClusteringConfig & config);

}  // namespace hybrid_localization