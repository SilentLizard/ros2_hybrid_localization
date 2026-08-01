#include "hybrid_localization_core/gaussian_fit_quality.hpp"

#include "hybrid_localization_core/detail/matrix3.hpp"
#include "hybrid_localization_core/geometry.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace hybrid_localization
{
namespace
{

constexpr double covariance_tolerance = 1e-12;
using Matrix3 = detail::Matrix3Storage;

[[nodiscard]] bool is_finite_pose(const Pose2d & pose) noexcept
{
  return
    std::isfinite(pose.x) &&
    std::isfinite(pose.y) &&
    std::isfinite(pose.yaw);
}

void validate_indices(
  const std::span<const WeightedParticle> particles,
  const std::span<const std::size_t> indices)
{
  if (indices.empty()) {
    throw std::invalid_argument(
            "Gaussian fit-quality evaluation requires particle indices");
  }

  std::vector<bool> seen(particles.size(), false);

  for (const std::size_t index : indices) {
    if (index >= particles.size()) {
      throw std::invalid_argument("Particle index is out of range");
    }

    if (seen[index]) {
      throw std::invalid_argument("Particle index occurs more than once");
    }

    seen[index] = true;
  }
}

void validate_covariance(const Matrix3 & covariance)
{
  detail::validate_covariance(
    covariance, covariance_tolerance, covariance_tolerance, "Gaussian covariance");
}

[[nodiscard]] Matrix3 regularized_cholesky_factor(
  const Matrix3 & covariance,
  const double regularization)
{
  detail::Matrix3 regularized = detail::matrix3_from_row_major(covariance);
  regularized.diagonal().array() += regularization;
  return detail::matrix3_to_row_major(regularized);
}

[[nodiscard]] double mahalanobis_distance(
  const Matrix3 & covariance,
  const std::array<double, 3> & residual)
{
  return std::sqrt(detail::mahalanobis_squared(
    residual, covariance, "Regularized Gaussian covariance"));
}

}  // namespace

GaussianFitQuality evaluate_gaussian_fit_quality(
  const std::span<const WeightedParticle> particles,
  const std::span<const std::size_t> indices,
  const GaussianComponent & component,
  const GaussianFitQualityConfig & config)
{
  if (
    !std::isfinite(config.covariance_regularization) ||
    config.covariance_regularization <= 0.0)
  {
    throw std::invalid_argument(
            "Covariance regularization must be finite and greater than zero");
  }

  if (!is_finite_pose(component.mean)) {
    throw std::invalid_argument("Gaussian component mean must be finite");
  }

  validate_covariance(component.covariance);

  const auto normalized = normalize_weights(particles);
  validate_indices(normalized, indices);

  double selected_weight = 0.0;
  for (const std::size_t index : indices) {
    selected_weight += normalized[index].weight;
  }

  if (!std::isfinite(selected_weight) || selected_weight <= 0.0) {
    throw std::invalid_argument(
            "Selected particles must have positive total weight");
  }

  const Matrix3 lower = regularized_cholesky_factor(
    component.covariance,
    config.covariance_regularization);

  GaussianFitQuality quality{};
  double weighted_sine = 0.0;
  double weighted_cosine = 0.0;

  for (const std::size_t index : indices) {
    const WeightedParticle & particle = normalized[index];
    const double conditional_weight = particle.weight / selected_weight;
    const double yaw_residual = angle_difference(
      particle.pose.yaw,
      component.mean.yaw);

    const std::array<double, 3> residual{
      particle.pose.x - component.mean.x,
      particle.pose.y - component.mean.y,
      yaw_residual};

    const double distance = mahalanobis_distance(lower, residual);

    quality.mean_mahalanobis_distance += conditional_weight * distance;
    quality.maximum_mahalanobis_distance =
      std::max(quality.maximum_mahalanobis_distance, distance);
    weighted_sine += conditional_weight * std::sin(particle.pose.yaw);
    weighted_cosine += conditional_weight * std::cos(particle.pose.yaw);
  }

  quality.angular_resultant_length =
    std::hypot(weighted_sine, weighted_cosine);

  /* Protect the documented [0, 1] range from roundoff. */
  quality.angular_resultant_length = std::clamp(
    quality.angular_resultant_length,
    0.0,
    1.0);

  return quality;
}

}  // namespace hybrid_localization