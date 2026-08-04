#include "hybrid_localization_core/gaussian_mixture_splitting.hpp"

#include "hybrid_localization_core/gaussian_statistics.hpp"
#include "hybrid_localization_core/detail/matrix3.hpp"
#include "hybrid_localization_core/geometry.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hybrid_localization
{
namespace
{

constexpr std::size_t dimension = 3U;

[[nodiscard]] bool finite_pose(const Pose2d & pose)
{
  return std::isfinite(pose.x) &&
         std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

void validate_config(const GaussianMixtureSplittingConfig & config)
{
  const auto valid_nonnegative_or_infinite = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };

  if (!valid_nonnegative_or_infinite(
      config.maximum_mean_mahalanobis_distance))
  {
    throw std::invalid_argument(
      "maximum_mean_mahalanobis_distance must be nonnegative");
  }

  if (!valid_nonnegative_or_infinite(
      config.maximum_fit_mahalanobis_distance))
  {
    throw std::invalid_argument(
      "maximum_fit_mahalanobis_distance must be nonnegative");
  }

  if (!std::isfinite(config.minimum_angular_resultant_length) ||
      config.minimum_angular_resultant_length < 0.0 ||
      config.minimum_angular_resultant_length > 1.0)
  {
    throw std::invalid_argument(
      "minimum_angular_resultant_length must lie in [0, 1]");
  }

  if (config.minimum_source_samples < 2U) {
    throw std::invalid_argument(
      "minimum_source_samples must be at least two");
  }

  if (config.minimum_child_samples == 0U) {
    throw std::invalid_argument(
      "minimum_child_samples must be positive");
  }

  if (config.minimum_source_samples < 2U * config.minimum_child_samples) {
    throw std::invalid_argument(
      "minimum_source_samples must permit two minimum-size children");
  }

  if (!std::isfinite(config.minimum_child_weight_fraction) ||
      config.minimum_child_weight_fraction <= 0.0 ||
      config.minimum_child_weight_fraction > 0.5)
  {
    throw std::invalid_argument(
      "minimum_child_weight_fraction must lie in (0, 0.5]");
  }

  if (config.maximum_splits == 0U) {
    throw std::invalid_argument("maximum_splits must be positive");
  }

  if (!std::isfinite(config.mass_tolerance) || config.mass_tolerance < 0.0) {
    throw std::invalid_argument(
      "mass_tolerance must be finite and nonnegative");
  }
}

void validate_component(const GaussianComponent & component)
{
  if (!finite_pose(component.mean)) {
    throw std::invalid_argument("Component mean must be finite");
  }
  if (!std::isfinite(component.weight) || component.weight <= 0.0) {
    throw std::invalid_argument("Component weight must be finite and positive");
  }
  constexpr double covariance_tolerance = 1e-12;
  detail::validate_covariance(
    component.covariance,
    covariance_tolerance,
    covariance_tolerance,
    "Component covariance");
}

void validate_mixture(
  const GaussianMixture & mixture,
  const double mass_tolerance)
{
  if (!std::isfinite(mixture.discarded_weight) ||
      mixture.discarded_weight < 0.0)
  {
    throw std::invalid_argument(
      "Discarded weight must be finite and nonnegative");
  }

  double total_mass = mixture.discarded_weight;
  for (const auto & component : mixture.components) {
    validate_component(component);
    total_mass += component.weight;
  }

  if (!std::isfinite(total_mass) ||
      std::abs(total_mass - 1.0) > mass_tolerance)
  {
    throw std::invalid_argument(
      "Mixture component and discarded weights must sum to one");
  }
}

void validate_fit_quality(const GaussianFitQuality & quality)
{
  if (!std::isfinite(quality.mean_mahalanobis_distance) ||
      quality.mean_mahalanobis_distance < 0.0 ||
      !std::isfinite(quality.maximum_mahalanobis_distance) ||
      quality.maximum_mahalanobis_distance < 0.0 ||
      !std::isfinite(quality.angular_resultant_length) ||
      quality.angular_resultant_length < 0.0 ||
      quality.angular_resultant_length > 1.0)
  {
    throw std::invalid_argument("Split fit-quality evidence is invalid");
  }
}

[[nodiscard]] bool should_split(
  const GaussianComponentSplitEvidence & evidence,
  const GaussianMixtureSplittingConfig & config)
{
  return evidence.fit_quality.mean_mahalanobis_distance >
           config.maximum_mean_mahalanobis_distance ||
         evidence.fit_quality.maximum_mahalanobis_distance >
           config.maximum_fit_mahalanobis_distance ||
         evidence.fit_quality.angular_resultant_length <
           config.minimum_angular_resultant_length;
}

[[nodiscard]] std::array<double, dimension> dominant_direction(
  const std::array<double, 9> & covariance)
{
  return detail::dominant_eigenvector(covariance, "Split component covariance");
}

struct ProjectedIndex
{
  std::size_t index{0U};
  double projection{0.0};
  double weight{0.0};
};

[[nodiscard]] std::pair<std::vector<std::size_t>, std::vector<std::size_t>>
partition_source_particles(
  const std::span<const WeightedParticle> normalized_particles,
  const GaussianComponent & component,
  const std::span<const std::size_t> source_indices,
  const GaussianMixtureSplittingConfig & config)
{
  const auto direction = dominant_direction(component.covariance);

  std::vector<ProjectedIndex> projected;
  projected.reserve(source_indices.size());
  double total_weight = 0.0;

  std::vector<bool> seen(normalized_particles.size(), false);
  for (const std::size_t index : source_indices) {
    if (index >= normalized_particles.size()) {
      throw std::invalid_argument("Split source index is out of range");
    }
    if (seen[index]) {
      throw std::invalid_argument("Split source index occurs more than once");
    }
    seen[index] = true;

    const auto & particle = normalized_particles[index];
    const std::array<double, dimension> residual{
      particle.pose.x - component.mean.x,
      particle.pose.y - component.mean.y,
      angle_difference(particle.pose.yaw, component.mean.yaw)};

    const double projection =
      residual[0] * direction[0] +
      residual[1] * direction[1] +
      residual[2] * direction[2];

    projected.push_back({index, projection, particle.weight});
    total_weight += particle.weight;
  }

  if (!std::isfinite(total_weight) || total_weight <= 0.0) {
    throw std::invalid_argument(
      "Split source particles must have positive total weight");
  }

  std::stable_sort(
    projected.begin(), projected.end(),
    [](const ProjectedIndex & lhs, const ProjectedIndex & rhs) {
      if (lhs.projection == rhs.projection) {
        return lhs.index < rhs.index;
      }
      return lhs.projection < rhs.projection;
    });

  const std::size_t minimum = config.minimum_child_samples;
  const std::size_t maximum_left = projected.size() - minimum;
  const double minimum_child_weight =
    config.minimum_child_weight_fraction * total_weight;

  std::size_t best_split = 0U;
  double best_balance_error = std::numeric_limits<double>::infinity();
  double left_weight = 0.0;

  for (std::size_t split = 1U; split < projected.size(); ++split) {
    left_weight += projected[split - 1U].weight;
    if (split < minimum || split > maximum_left) {
      continue;
    }

    const double right_weight = total_weight - left_weight;
    if (left_weight + 1e-15 < minimum_child_weight ||
        right_weight + 1e-15 < minimum_child_weight)
    {
      continue;
    }

    const double balance_error =
      std::abs(left_weight - 0.5 * total_weight);
    if (balance_error < best_balance_error) {
      best_balance_error = balance_error;
      best_split = split;
    }
  }

  if (best_split == 0U) {
    throw std::domain_error(
      "Source particles cannot satisfy child split constraints");
  }

  std::vector<std::size_t> left;
  std::vector<std::size_t> right;
  left.reserve(best_split);
  right.reserve(projected.size() - best_split);

  for (std::size_t index = 0U; index < projected.size(); ++index) {
    if (index < best_split) {
      left.push_back(projected[index].index);
    } else {
      right.push_back(projected[index].index);
    }
  }

  return {std::move(left), std::move(right)};
}

[[nodiscard]] std::pair<GaussianComponent, GaussianComponent> split_component(
  const std::span<const WeightedParticle> normalized_particles,
  const GaussianComponent & parent,
  const GaussianComponentSplitEvidence & evidence,
  HypothesisIdGenerator & id_generator,
  const GaussianMixtureSplittingConfig & config)
{
  const auto [left_indices, right_indices] = partition_source_particles(
    normalized_particles, parent, evidence.source_indices, config);

  GaussianComponent left = fit_gaussian(normalized_particles, left_indices);
  GaussianComponent right = fit_gaussian(normalized_particles, right_indices);

  const double source_mass = left.weight + right.weight;
  if (!std::isfinite(source_mass) || source_mass <= 0.0) {
    throw std::domain_error("Split children have invalid source mass");
  }

  const double scale = parent.weight / source_mass;
  left.weight *= scale;
  right.weight = parent.weight - left.weight;
  left.provenance = make_split_provenance(id_generator, parent.provenance);
  right.provenance = make_split_provenance(id_generator, parent.provenance);

  return {std::move(left), std::move(right)};
}

}  // namespace

GaussianMixtureSplittingResult split_gaussian_mixture_components(
  const std::span<const WeightedParticle> particles,
  const GaussianMixture & mixture,
  const std::span<const GaussianComponentSplitEvidence> evidence,
  HypothesisIdGenerator & id_generator,
  const GaussianMixtureSplittingConfig & config)
{
  validate_config(config);
  validate_mixture(mixture, config.mass_tolerance);

  if (evidence.size() != mixture.components.size()) {
    throw std::invalid_argument(
      "Split evidence count must match mixture component count");
  }

  const auto normalized_particles = normalize_weights(particles);

  GaussianMixtureSplittingResult result{};
  result.mixture.discarded_weight = mixture.discarded_weight;
  result.mixture.components.reserve(
    mixture.components.size() + config.maximum_splits);

  for (std::size_t component_index = 0U;
    component_index < mixture.components.size(); ++component_index)
  {
    const auto & component = mixture.components[component_index];
    const auto & component_evidence = evidence[component_index];
    validate_fit_quality(component_evidence.fit_quality);

    const bool eligible =
      result.split_count < config.maximum_splits &&
      component_evidence.source_indices.size() >=
        config.minimum_source_samples &&
      should_split(component_evidence, config);

    if (!eligible) {
      result.mixture.components.push_back(component);
      continue;
    }

    const auto [left, right] = split_component(
      normalized_particles, component, component_evidence, id_generator, config);
    result.created_hypothesis_ids.push_back(left.provenance.id);
    result.created_hypothesis_ids.push_back(right.provenance.id);
    result.mixture.components.push_back(left);
    result.mixture.components.push_back(right);
    result.split_component_indices.push_back(component_index);
    ++result.split_count;
  }

  std::stable_sort(
    result.mixture.components.begin(), result.mixture.components.end(),
    [](const GaussianComponent & lhs, const GaussianComponent & rhs) {
      return lhs.weight > rhs.weight;
    });

  double total_mass = result.mixture.discarded_weight;
  for (const auto & component : result.mixture.components) {
    total_mass += component.weight;
  }
  if (std::abs(total_mass - 1.0) > config.mass_tolerance) {
    throw std::logic_error("Splitting failed to preserve mixture mass");
  }

  return result;
}

GaussianMixtureSplittingResult split_gaussian_mixture_components(
  const std::span<const WeightedParticle> particles,
  const GaussianMixture & mixture,
  const std::span<const GaussianComponentSplitEvidence> evidence,
  const GaussianMixtureSplittingConfig & config)
{
  HypothesisId next_id = 1U;
  for (const auto & component : mixture.components) {
    if (has_hypothesis_id(component.provenance)) {
      next_id = std::max(next_id, component.provenance.id + 1U);
    }
  }
  HypothesisIdGenerator id_generator(next_id);
  return split_gaussian_mixture_components(
    particles, mixture, evidence, id_generator, config);
}

}  // namespace hybrid_localization
