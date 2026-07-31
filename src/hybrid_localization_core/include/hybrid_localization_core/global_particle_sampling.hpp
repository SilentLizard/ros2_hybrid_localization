#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Policy for occupancy-grid cells whose value is unknown (-1).
enum class UnknownCellPolicy
{
  exclude,
  include
};

/// Non-owning, ROS-independent view of a planar occupancy grid.
///
/// Cell data is row-major. Occupancy values must lie in [-1, 100], where -1
/// denotes unknown space. The origin pose describes the world pose of the
/// lower-left corner of cell (0, 0).
struct OccupancyGridView
{
  std::size_t width{0U};
  std::size_t height{0U};
  double resolution{0.0};
  Pose2d origin{};
  std::span<const std::int8_t> cells{};
};

/// Configuration for map-aware global particle generation.
struct GlobalParticleSamplingConfig
{
  /// Exact number of particles returned by the sampler.
  std::size_t particle_count{0U};

  /// Maximum occupancy value accepted for sampling.
  std::int8_t maximum_occupancy{0};

  /// Whether unknown cells may be sampled.
  UnknownCellPolicy unknown_cell_policy{UnknownCellPolicy::exclude};

  /// Seed used by the deterministic pseudo-random number generator.
  std::uint64_t random_seed{0U};
};

/// Generate an equally weighted global particle belief from eligible map cells.
///
/// Eligible cells are selected uniformly. Position is then sampled uniformly
/// inside the chosen cell, and yaw is sampled uniformly from [-pi, pi).
[[nodiscard]] std::vector<WeightedParticle> sample_global_particles(
  const OccupancyGridView & grid,
  const GlobalParticleSamplingConfig & config);

}  // namespace hybrid_localization