#include "hybrid_localization_core/gaussian_mixture_merging.hpp"

#include "hybrid_localization_core/detail/matrix3.hpp"
#include "hybrid_localization_core/geometry.hpp"

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

using Matrix3 = detail::Matrix3Storage;
using Vector3 = detail::Vector3Storage;

[[nodiscard]] double & at(Matrix3 & matrix, const std::size_t row, const std::size_t column)
{
  return matrix[row * 3U + column];
}

[[nodiscard]] double at(
  const Matrix3 & matrix,
  const std::size_t row,
  const std::size_t column)
{
  return matrix[row * 3U + column];
}

void validate_config(const GaussianMixtureMergingConfig & config)
{
  if (std::isnan(config.maximum_mahalanobis_distance_squared) ||
    config.maximum_mahalanobis_distance_squared < 0.0)
  {
    throw std::invalid_argument(
            "Maximum Mahalanobis distance squared must be nonnegative");
  }

  if (std::isnan(config.maximum_yaw_difference) ||
    config.maximum_yaw_difference < 0.0)
  {
    throw std::invalid_argument(
            "Maximum yaw difference must be nonnegative");
  }

  if (!std::isfinite(config.covariance_regularization) ||
    config.covariance_regularization <= 0.0)
  {
    throw std::invalid_argument(
            "Covariance regularization must be finite and positive");
  }

  if (!std::isfinite(config.covariance_symmetry_tolerance) ||
    config.covariance_symmetry_tolerance < 0.0 ||
    !std::isfinite(config.covariance_psd_tolerance) ||
    config.covariance_psd_tolerance < 0.0 ||
    !std::isfinite(config.mass_tolerance) ||
    config.mass_tolerance < 0.0)
  {
    throw std::invalid_argument(
            "Numerical tolerances must be finite and nonnegative");
  }
}

[[nodiscard]] double determinant(const Matrix3 & matrix)
{
  return detail::determinant(matrix);
}

void validate_covariance(
  const Matrix3 & covariance,
  const GaussianMixtureMergingConfig & config)
{
  detail::validate_covariance(
    covariance,
    config.covariance_symmetry_tolerance,
    config.covariance_psd_tolerance,
    "Gaussian covariance");
}

void validate_mixture(
  const GaussianMixture & mixture,
  const GaussianMixtureMergingConfig & config)
{
  if (!std::isfinite(mixture.discarded_weight) ||
    mixture.discarded_weight < 0.0)
  {
    throw std::invalid_argument(
            "Discarded mixture weight must be finite and nonnegative");
  }

  double total_mass = mixture.discarded_weight;
  for (const auto & component : mixture.components) {
    if (!std::isfinite(component.mean.x) ||
      !std::isfinite(component.mean.y) ||
      !std::isfinite(component.mean.yaw))
    {
      throw std::invalid_argument("Gaussian component mean must be finite");
    }

    if (!std::isfinite(component.weight) || component.weight <= 0.0) {
      throw std::invalid_argument(
              "Gaussian component weight must be finite and positive");
    }

    validate_covariance(component.covariance, config);
    total_mass += component.weight;
  }

  if (!std::isfinite(total_mass) ||
    std::abs(total_mass - 1.0) > config.mass_tolerance)
  {
    throw std::invalid_argument(
            "Gaussian mixture probability mass must sum to one");
  }
}

[[nodiscard]] Matrix3 inverse(const Matrix3 & matrix)
{
  return detail::inverse_positive_definite(matrix, "Regularized merge covariance");
}

[[nodiscard]] double quadratic_form(
  const Vector3 & vector,
  const Matrix3 & matrix)
{
  return detail::quadratic_form(vector, matrix);
}

[[nodiscard]] double merge_distance_squared(
  const GaussianComponent & lhs,
  const GaussianComponent & rhs,
  const GaussianMixtureMergingConfig & config)
{
  Matrix3 covariance{};
  for (std::size_t index = 0U; index < covariance.size(); ++index) {
    covariance[index] = lhs.covariance[index] + rhs.covariance[index];
  }
  at(covariance, 0U, 0U) += config.covariance_regularization;
  at(covariance, 1U, 1U) += config.covariance_regularization;
  at(covariance, 2U, 2U) += config.covariance_regularization;

  const Vector3 delta{
    rhs.mean.x - lhs.mean.x,
    rhs.mean.y - lhs.mean.y,
    angle_difference(rhs.mean.yaw, lhs.mean.yaw)};

  const double distance = quadratic_form(delta, inverse(covariance));
  return std::max(0.0, distance);
}

[[nodiscard]] GaussianComponent merge_pair(
  const GaussianComponent & lhs,
  const GaussianComponent & rhs,
  HypothesisIdGenerator & id_generator)
{
  const double merged_weight = lhs.weight + rhs.weight;
  const double lhs_fraction = lhs.weight / merged_weight;
  const double rhs_fraction = rhs.weight / merged_weight;

  if (lhs.sample_count >
    std::numeric_limits<std::size_t>::max() - rhs.sample_count)
  {
    throw std::overflow_error("Merged Gaussian sample count overflows");
  }

  GaussianComponent merged;
  merged.weight = merged_weight;
  merged.sample_count = lhs.sample_count + rhs.sample_count;
  merged.provenance = make_merged_provenance(
    id_generator, lhs.provenance, rhs.provenance);
  merged.mean.x = lhs_fraction * lhs.mean.x + rhs_fraction * rhs.mean.x;
  merged.mean.y = lhs_fraction * lhs.mean.y + rhs_fraction * rhs.mean.y;

  const double sine = lhs_fraction * std::sin(lhs.mean.yaw) +
    rhs_fraction * std::sin(rhs.mean.yaw);
  const double cosine = lhs_fraction * std::cos(lhs.mean.yaw) +
    rhs_fraction * std::cos(rhs.mean.yaw);
  merged.mean.yaw = normalize_angle(std::atan2(sine, cosine));

  const Vector3 lhs_delta{
    lhs.mean.x - merged.mean.x,
    lhs.mean.y - merged.mean.y,
    angle_difference(lhs.mean.yaw, merged.mean.yaw)};
  const Vector3 rhs_delta{
    rhs.mean.x - merged.mean.x,
    rhs.mean.y - merged.mean.y,
    angle_difference(rhs.mean.yaw, merged.mean.yaw)};

  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      at(merged.covariance, row, column) =
        lhs_fraction *
        (at(lhs.covariance, row, column) +
        lhs_delta[row] * lhs_delta[column]) +
        rhs_fraction *
        (at(rhs.covariance, row, column) +
        rhs_delta[row] * rhs_delta[column]);
    }
  }

  return merged;
}

struct MergeCandidate
{
  std::size_t first{0U};
  std::size_t second{0U};
  double distance_squared{std::numeric_limits<double>::infinity()};
  bool valid{false};
};

[[nodiscard]] MergeCandidate find_best_candidate(
  const std::vector<GaussianComponent> & components,
  const GaussianMixtureMergingConfig & config)
{
  MergeCandidate best;
  for (std::size_t first = 0U; first < components.size(); ++first) {
    for (std::size_t second = first + 1U; second < components.size(); ++second) {
      const double yaw_difference = std::abs(angle_difference(
          components[second].mean.yaw,
          components[first].mean.yaw));
      if (yaw_difference > config.maximum_yaw_difference) {
        continue;
      }

      const double distance_squared = merge_distance_squared(
        components[first], components[second], config);
      if (distance_squared > config.maximum_mahalanobis_distance_squared) {
        continue;
      }

      if (!best.valid || distance_squared < best.distance_squared) {
        best = MergeCandidate{first, second, distance_squared, true};
      }
    }
  }
  return best;
}

}  // namespace

GaussianMixtureMergingResult merge_gaussian_mixture_components(
  const GaussianMixture & mixture,
  HypothesisIdGenerator & id_generator,
  const GaussianMixtureMergingConfig & config)
{
  validate_config(config);
  validate_mixture(mixture, config);

  GaussianMixtureMergingResult result;
  result.mixture = mixture;

  while (result.mixture.components.size() >= 2U) {
    const MergeCandidate candidate = find_best_candidate(
      result.mixture.components, config);
    if (!candidate.valid) {
      break;
    }

    GaussianComponent merged = merge_pair(
      result.mixture.components[candidate.first],
      result.mixture.components[candidate.second],
      id_generator);
    result.created_hypothesis_ids.push_back(merged.provenance.id);

    result.mixture.components[candidate.first] = std::move(merged);
    result.mixture.components.erase(
      result.mixture.components.begin() +
      static_cast<std::ptrdiff_t>(candidate.second));
    ++result.merge_count;
  }

  std::stable_sort(
    result.mixture.components.begin(),
    result.mixture.components.end(),
    [](const GaussianComponent & lhs, const GaussianComponent & rhs) {
      return lhs.weight > rhs.weight;
    });

  const double component_mass = std::accumulate(
    result.mixture.components.begin(),
    result.mixture.components.end(),
    0.0,
    [](const double total, const GaussianComponent & component) {
      return total + component.weight;
    });
  const double total_mass = component_mass + result.mixture.discarded_weight;
  if (!std::isfinite(total_mass) ||
    std::abs(total_mass - 1.0) > config.mass_tolerance)
  {
    throw std::runtime_error(
            "Merged Gaussian mixture probability mass is inconsistent");
  }

  return result;
}

GaussianMixtureMergingResult merge_gaussian_mixture_components(
  const GaussianMixture & mixture,
  const GaussianMixtureMergingConfig & config)
{
  HypothesisId next_id = 1U;
  for (const auto & component : mixture.components) {
    if (has_hypothesis_id(component.provenance)) {
      next_id = std::max(next_id, component.provenance.id + 1U);
    }
  }
  HypothesisIdGenerator id_generator(next_id);
  return merge_gaussian_mixture_components(mixture, id_generator, config);
}

}  // namespace hybrid_localization