#include "hybrid_localization_core/gaussian_fit_quality.hpp"

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
using Matrix3 = std::array<double, 9>;

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
  for (const double value : covariance) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
              "Gaussian covariance must contain only finite values");
    }
  }

  if (
    std::abs(covariance[1] - covariance[3]) > covariance_tolerance ||
    std::abs(covariance[2] - covariance[6]) > covariance_tolerance ||
    std::abs(covariance[5] - covariance[7]) > covariance_tolerance)
  {
    throw std::invalid_argument("Gaussian covariance must be symmetric");
  }
}

[[nodiscard]] Matrix3 regularized_cholesky_factor(
  const Matrix3 & covariance,
  const double regularization)
{
  Matrix3 lower{};

  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column <= row; ++column) {
      double value = covariance[row * 3U + column];

      if (row == column) {
        value += regularization;
      }

      for (std::size_t k = 0U; k < column; ++k) {
        value -=
          lower[row * 3U + k] *
          lower[column * 3U + k];
      }

      if (row == column) {
        if (!std::isfinite(value) || value <= 0.0) {
          throw std::invalid_argument(
                  "Regularized Gaussian covariance must be positive definite");
        }

        lower[row * 3U + column] = std::sqrt(value);
      } else {
        lower[row * 3U + column] =
          value / lower[column * 3U + column];
      }
    }
  }

  return lower;
}

[[nodiscard]] double mahalanobis_distance(
  const Matrix3 & lower,
  const std::array<double, 3> & residual)
{
  std::array<double, 3> whitened{};

  for (std::size_t row = 0U; row < 3U; ++row) {
    double value = residual[row];

    for (std::size_t column = 0U; column < row; ++column) {
      value -= lower[row * 3U + column] * whitened[column];
    }

    whitened[row] = value / lower[row * 3U + row];
  }

  double squared_distance = 0.0;
  for (const double value : whitened) {
    squared_distance += value * value;
  }

  return std::sqrt(std::max(0.0, squared_distance));
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