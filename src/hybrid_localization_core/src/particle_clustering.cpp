#include "hybrid_localization_core/particle_clustering.hpp"

#include "hybrid_localization_core/geometry.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <vector>

namespace hybrid_localization
{

// Public function: declared in particle_clustering.hpp.
void validate_particle_clustering_config(
  const ParticleClusteringConfig & config)
{
  if (!std::isfinite(config.position_scale) ||
      config.position_scale <= 0.0)
  {
    throw std::invalid_argument(
      "position_scale must be positive and finite");
  }

  if (!std::isfinite(config.yaw_scale) ||
      config.yaw_scale <= 0.0)
  {
    throw std::invalid_argument(
      "yaw_scale must be positive and finite");
  }

  if (!std::isfinite(config.epsilon) ||
      config.epsilon <= 0.0)
  {
    throw std::invalid_argument(
      "epsilon must be positive and finite");
  }

  if (config.minimum_neighbors == 0U) {
    throw std::invalid_argument(
      "minimum_neighbors must be at least one");
  }

  if (!std::isfinite(config.minimum_core_weight) ||
      config.minimum_core_weight < 0.0 ||
      config.minimum_core_weight > 1.0)
  {
    throw std::invalid_argument(
      "minimum_core_weight must be finite and in [0, 1]");
  }

  if (!std::isfinite(config.minimum_cluster_weight) ||
      config.minimum_cluster_weight < 0.0 ||
      config.minimum_cluster_weight > 1.0)
  {
    throw std::invalid_argument(
      "minimum_cluster_weight must be finite and in [0, 1]");
  }
}

namespace
{

constexpr int unvisited_label = -2;
constexpr int noise_label = -1;

/*
 * This must appear before find_neighbors(), because find_neighbors()
 * calls it.
 */
[[nodiscard]] double particle_distance_squared_unchecked(
  const Pose2d & lhs,
  const Pose2d & rhs,
  const ParticleClusteringConfig & config)
{
  const double delta_x =
    (lhs.x - rhs.x) / config.position_scale;

  const double delta_y =
    (lhs.y - rhs.y) / config.position_scale;

  const double delta_yaw =
    angle_difference(lhs.yaw, rhs.yaw) / config.yaw_scale;

  return
    delta_x * delta_x +
    delta_y * delta_y +
    delta_yaw * delta_yaw;
}

[[nodiscard]] std::vector<std::size_t> find_neighbors(
  const std::span<const WeightedParticle> particles,
  const std::size_t center_index,
  const ParticleClusteringConfig & config)
{
  std::vector<std::size_t> neighbors;

  const double epsilon_squared =
    config.epsilon * config.epsilon;

  for (std::size_t index = 0; index < particles.size(); ++index) {
    const double distance_squared =
      particle_distance_squared_unchecked(
        particles[center_index].pose,
        particles[index].pose,
        config);

    if (distance_squared <= epsilon_squared) {
      neighbors.push_back(index);
    }
  }

  return neighbors;
}

[[nodiscard]] double neighborhood_weight(
  const std::span<const WeightedParticle> normalized_particles,
  const std::span<const std::size_t> neighbors)
{
  double weight = 0.0;

  for (const std::size_t index : neighbors) {
    weight += normalized_particles[index].weight;
  }

  return weight;
}

[[nodiscard]] bool is_core_neighborhood(
  const std::span<const WeightedParticle> normalized_particles,
  const std::span<const std::size_t> neighbors,
  const ParticleClusteringConfig & config)
{
  if (neighbors.size() < config.minimum_neighbors) {
    return false;
  }

  return neighborhood_weight(normalized_particles, neighbors) >=
         config.minimum_core_weight;
}

void append_unique(
  std::deque<std::size_t> & pending,
  std::vector<bool> & queued,
  const std::span<const std::size_t> candidates)
{
  for (const std::size_t index : candidates) {
    if (!queued[index]) {
      pending.push_back(index);
      queued[index] = true;
    }
  }
}

}  // namespace

double particle_distance_squared(
  const Pose2d & lhs,
  const Pose2d & rhs,
  const ParticleClusteringConfig & config)
{
  validate_particle_clustering_config(config);

  return particle_distance_squared_unchecked(
    lhs,
    rhs,
    config);
}

ParticleClusteringResult cluster_particles(
  const std::span<const WeightedParticle> particles,
  const ParticleClusteringConfig & config)
{
  validate_particle_clustering_config(config);

  /*
   * normalize_weights() also validates that the set is nonempty and that
   * weights and pose values are finite and valid.
   */
  const auto normalized = normalize_weights(particles);

  std::vector<int> labels(
    normalized.size(),
    unvisited_label);

  int next_cluster_label = 0;

  for (std::size_t particle_index = 0;
       particle_index < normalized.size();
       ++particle_index)
  {
    if (labels[particle_index] != unvisited_label) {
      continue;
    }

    const auto initial_neighbors = find_neighbors(
      normalized,
      particle_index,
      config);

    if (!is_core_neighborhood(
          normalized,
          initial_neighbors,
          config))
    {
      labels[particle_index] = noise_label;
      continue;
    }

    const int current_cluster_label = next_cluster_label++;
    labels[particle_index] = current_cluster_label;

    std::deque<std::size_t> pending;
    std::vector<bool> queued(normalized.size(), false);

    append_unique(
      pending,
      queued,
      initial_neighbors);

    while (!pending.empty()) {
      const std::size_t candidate_index = pending.front();
      pending.pop_front();

      /*
       * In DBSCAN, an earlier noise point may later become a border point
       * when reached from a valid core neighborhood.
       */
      if (labels[candidate_index] == noise_label) {
        labels[candidate_index] = current_cluster_label;
      }

      if (labels[candidate_index] != unvisited_label) {
        continue;
      }

      labels[candidate_index] = current_cluster_label;

      const auto candidate_neighbors = find_neighbors(
        normalized,
        candidate_index,
        config);

      if (is_core_neighborhood(
            normalized,
            candidate_neighbors,
            config))
      {
        append_unique(
          pending,
          queued,
          candidate_neighbors);
      }
    }
  }

  std::vector<ParticleCluster> provisional_clusters(
    static_cast<std::size_t>(next_cluster_label));

  for (std::size_t index = 0; index < labels.size(); ++index) {
    const int label = labels[index];

    if (label < 0) {
      continue;
    }

    auto & cluster =
      provisional_clusters[static_cast<std::size_t>(label)];

    cluster.particle_indices.push_back(index);
    cluster.weight += normalized[index].weight;
  }

  ParticleClusteringResult result;
  std::vector<bool> retained(normalized.size(), false);

  for (auto & cluster : provisional_clusters) {
    if (cluster.weight < config.minimum_cluster_weight) {
      continue;
    }

    for (const std::size_t index : cluster.particle_indices) {
      retained[index] = true;
    }

    result.clusters.push_back(std::move(cluster));
  }

  std::sort(
    result.clusters.begin(),
    result.clusters.end(),
    [](const ParticleCluster & lhs, const ParticleCluster & rhs) {
      return lhs.weight > rhs.weight;
    });

  for (std::size_t index = 0; index < normalized.size(); ++index) {
    if (!retained[index]) {
      result.noise_indices.push_back(index);
    }
  }

  return result;
}

}  // namespace hybrid_localization