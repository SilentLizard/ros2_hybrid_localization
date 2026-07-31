#include "hybrid_localization_core/global_particle_sampling.hpp"

#include "hybrid_localization_core/geometry.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

namespace hybrid_localization
{
namespace
{

[[nodiscard]] bool is_finite_pose(const Pose2d & pose) noexcept
{
  return
    std::isfinite(pose.x) &&
    std::isfinite(pose.y) &&
    std::isfinite(pose.yaw);
}

[[nodiscard]] std::size_t checked_cell_count(
  const std::size_t width,
  const std::size_t height)
{
  if (width == 0U || height == 0U) {
    throw std::invalid_argument(
            "Occupancy grid width and height must be greater than zero");
  }

  if (width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::invalid_argument("Occupancy grid dimensions overflow");
  }

  return width * height;
}

[[nodiscard]] bool is_eligible_cell(
  const std::int8_t occupancy,
  const GlobalParticleSamplingConfig & config)
{
  if (occupancy == static_cast<std::int8_t>(-1)) {
    return config.unknown_cell_policy == UnknownCellPolicy::include;
  }

  return occupancy <= config.maximum_occupancy;
}

}  // namespace

std::vector<WeightedParticle> sample_global_particles(
  const OccupancyGridView & grid,
  const GlobalParticleSamplingConfig & config)
{
  if (config.particle_count == 0U) {
    throw std::invalid_argument(
            "Global particle count must be greater than zero");
  }

  if (
    config.maximum_occupancy < static_cast<std::int8_t>(0) ||
    config.maximum_occupancy > static_cast<std::int8_t>(100))
  {
    throw std::invalid_argument(
            "Maximum occupancy must lie in [0, 100]");
  }

  if (!std::isfinite(grid.resolution) || grid.resolution <= 0.0) {
    throw std::invalid_argument(
            "Occupancy grid resolution must be finite and greater than zero");
  }

  if (!is_finite_pose(grid.origin)) {
    throw std::invalid_argument("Occupancy grid origin must be finite");
  }

  const std::size_t cell_count = checked_cell_count(grid.width, grid.height);

  if (grid.cells.size() != cell_count) {
    throw std::invalid_argument(
            "Occupancy grid cell count does not match its dimensions");
  }

  std::vector<std::size_t> eligible_indices;
  eligible_indices.reserve(cell_count);

  for (std::size_t index = 0U; index < cell_count; ++index) {
    const std::int8_t occupancy = grid.cells[index];

    if (
      occupancy < static_cast<std::int8_t>(-1) ||
      occupancy > static_cast<std::int8_t>(100))
    {
      throw std::invalid_argument(
              "Occupancy grid values must lie in [-1, 100]");
    }

    if (is_eligible_cell(occupancy, config)) {
      eligible_indices.push_back(index);
    }
  }

  if (eligible_indices.empty()) {
    throw std::invalid_argument(
            "Occupancy grid contains no cells eligible for global sampling");
  }

  std::mt19937_64 random_engine(config.random_seed);
  std::uniform_int_distribution<std::size_t> cell_distribution(
    0U,
    eligible_indices.size() - 1U);
  std::uniform_real_distribution<double> unit_distribution(0.0, 1.0);
  std::uniform_real_distribution<double> yaw_distribution(
    -std::numbers::pi,
    std::numbers::pi);

  const double cosine = std::cos(grid.origin.yaw);
  const double sine = std::sin(grid.origin.yaw);
  const double particle_weight =
    1.0 / static_cast<double>(config.particle_count);

  std::vector<WeightedParticle> particles;
  particles.reserve(config.particle_count);

  for (std::size_t sample = 0U; sample < config.particle_count; ++sample) {
    const std::size_t cell_index =
      eligible_indices[cell_distribution(random_engine)];
    const std::size_t row = cell_index / grid.width;
    const std::size_t column = cell_index % grid.width;

    const double local_x =
      (static_cast<double>(column) + unit_distribution(random_engine)) *
      grid.resolution;
    const double local_y =
      (static_cast<double>(row) + unit_distribution(random_engine)) *
      grid.resolution;

    const double world_x =
      grid.origin.x + cosine * local_x - sine * local_y;
    const double world_y =
      grid.origin.y + sine * local_x + cosine * local_y;

    particles.push_back(
      {
        {
          world_x,
          world_y,
          normalize_angle(yaw_distribution(random_engine))
        },
        particle_weight
      });
  }

  return particles;
}

}  // namespace hybrid_localization