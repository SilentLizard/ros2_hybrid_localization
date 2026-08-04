#include "hybrid_localization_core/particle_gmm_comparison.hpp"

#include "hybrid_localization_core/detail/matrix3.hpp"
#include "hybrid_localization_core/gaussian_statistics.hpp"
#include "hybrid_localization_core/geometry.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace hybrid_localization
{
namespace
{

using Matrix3 = detail::Matrix3Storage;
using Vector3 = detail::Vector3Storage;


void validate_covariance(
  const Matrix3 & covariance,
  const double symmetry_tolerance,
  const double psd_tolerance)
{
  detail::validate_covariance(covariance, symmetry_tolerance, psd_tolerance, "Covariance");
}

[[nodiscard]] Matrix3 add_regularized(
  const Matrix3 & lhs,
  const Matrix3 & rhs,
  const double regularization)
{
  detail::Matrix3 result =
    detail::matrix3_from_row_major(lhs) + detail::matrix3_from_row_major(rhs);
  result.diagonal().array() += regularization;
  return detail::matrix3_to_row_major(result);
}

[[nodiscard]] Vector3 solve_3x3(
  const Matrix3 & matrix,
  const Vector3 & rhs)
{
  return detail::solve_positive_definite(matrix, rhs, "Regularized covariance");
}

[[nodiscard]] double mahalanobis_squared(
  const Vector3 & residual,
  const Matrix3 & covariance)
{
  const Vector3 solved = solve_3x3(covariance, residual);
  double value = 0.0;
  for (std::size_t index = 0; index < 3; ++index) {
    value += residual[index] * solved[index];
  }
  if (!std::isfinite(value)) {
    throw std::domain_error("Mahalanobis distance is not finite");
  }
  return std::max(0.0, value);
}

void validate_config(const ParticleGmmComparisonConfig & config)
{
  if (!std::isfinite(config.covariance_regularization) ||
      config.covariance_regularization <= 0.0)
  {
    throw std::invalid_argument("Covariance regularization must be finite and positive");
  }
  if (!std::isfinite(config.particle_support_mahalanobis_distance_squared) ||
      config.particle_support_mahalanobis_distance_squared < 0.0)
  {
    throw std::invalid_argument("Particle support gate must be finite and nonnegative");
  }
  if (!std::isfinite(config.covariance_symmetry_tolerance) ||
      config.covariance_symmetry_tolerance < 0.0 ||
      !std::isfinite(config.covariance_psd_tolerance) ||
      config.covariance_psd_tolerance < 0.0 ||
      !std::isfinite(config.mass_tolerance) ||
      config.mass_tolerance < 0.0)
  {
    throw std::invalid_argument("Numerical tolerances must be finite and nonnegative");
  }
}

void validate_pose(const Pose2d & pose)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    throw std::invalid_argument("Pose values must be finite");
  }
}

[[nodiscard]] GaussianComponent moment_match_mixture(
  const GaussianMixture & mixture,
  const ParticleGmmComparisonConfig & config)
{
  double represented_weight = 0.0;
  double weighted_x = 0.0;
  double weighted_y = 0.0;
  double weighted_sine = 0.0;
  double weighted_cosine = 0.0;

  for (const auto & component : mixture.components) {
    validate_pose(component.mean);
    if (!std::isfinite(component.weight) || component.weight <= 0.0) {
      throw std::invalid_argument("GMM component weights must be finite and positive");
    }
    validate_covariance(component.covariance, config.covariance_symmetry_tolerance,
      config.covariance_psd_tolerance);
    represented_weight += component.weight;
    weighted_x += component.weight * component.mean.x;
    weighted_y += component.weight * component.mean.y;
    weighted_sine += component.weight * std::sin(component.mean.yaw);
    weighted_cosine += component.weight * std::cos(component.mean.yaw);
  }

  if (!std::isfinite(mixture.discarded_weight) || mixture.discarded_weight < 0.0) {
    throw std::invalid_argument("Discarded GMM weight must be finite and nonnegative");
  }
  if (std::abs(represented_weight + mixture.discarded_weight - 1.0) > config.mass_tolerance) {
    throw std::invalid_argument("GMM probability mass must sum to one");
  }
  if (represented_weight <= 0.0) {
    throw std::invalid_argument("Shadow comparison requires represented GMM mass");
  }

  weighted_x /= represented_weight;
  weighted_y /= represented_weight;
  weighted_sine /= represented_weight;
  weighted_cosine /= represented_weight;
  if (std::hypot(weighted_sine, weighted_cosine) < 1e-12) {
    throw std::domain_error("GMM circular mean is undefined");
  }

  GaussianComponent result{};
  result.mean = {weighted_x, weighted_y, std::atan2(weighted_sine, weighted_cosine)};
  result.weight = represented_weight;

  for (const auto & component : mixture.components) {
    const double alpha = component.weight / represented_weight;
    const Vector3 delta{
      component.mean.x - result.mean.x,
      component.mean.y - result.mean.y,
      angle_difference(component.mean.yaw, result.mean.yaw)};

    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        result.covariance[row * 3 + column] += alpha *
          (component.covariance[row * 3 + column] + delta[row] * delta[column]);
      }
    }
  }
  return result;
}

[[nodiscard]] Vector3 pose_residual(const Pose2d & lhs, const Pose2d & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, angle_difference(lhs.yaw, rhs.yaw)};
}

}  // namespace

ParticleGmmComparison compare_particle_and_gmm_beliefs(
  const std::span<const WeightedParticle> particles,
  const GaussianMixture & mixture,
  const ParticleGmmComparisonConfig & config)
{
  validate_config(config);
  const auto normalized_particles = normalize_weights(particles);
  const GaussianComponent particle_gaussian = fit_gaussian(normalized_particles);
  validate_covariance(particle_gaussian.covariance, config.covariance_symmetry_tolerance,
    config.covariance_psd_tolerance);
  const GaussianComponent gmm_gaussian = moment_match_mixture(mixture, config);

  ParticleGmmComparison result{};
  result.particle_mean = particle_gaussian.mean;
  result.particle_covariance = particle_gaussian.covariance;
  result.gmm_mean = gmm_gaussian.mean;
  result.gmm_covariance = gmm_gaussian.covariance;
  result.represented_gmm_weight = gmm_gaussian.weight;
  result.discarded_gmm_weight = mixture.discarded_weight;

  const Vector3 belief_residual = pose_residual(result.particle_mean, result.gmm_mean);
  result.position_difference = std::hypot(belief_residual[0], belief_residual[1]);
  result.absolute_yaw_difference = std::abs(belief_residual[2]);
  result.belief_mahalanobis_distance_squared = mahalanobis_squared(
    belief_residual,
    add_regularized(particle_gaussian.covariance, gmm_gaussian.covariance,
      config.covariance_regularization));

  const auto dominant_iterator = std::max_element(
    mixture.components.begin(), mixture.components.end(),
    [](const GaussianComponent & lhs, const GaussianComponent & rhs) {
      return lhs.weight < rhs.weight;
    });
  result.dominant_component_index = static_cast<std::size_t>(
    std::distance(mixture.components.begin(), dominant_iterator));
  const Vector3 dominant_residual = pose_residual(
    particle_gaussian.mean, dominant_iterator->mean);
  result.dominant_component_position_difference =
    std::hypot(dominant_residual[0], dominant_residual[1]);
  result.dominant_component_absolute_yaw_difference = std::abs(dominant_residual[2]);
  result.dominant_component_mahalanobis_distance_squared = mahalanobis_squared(
    dominant_residual,
    add_regularized(particle_gaussian.covariance, dominant_iterator->covariance,
      config.covariance_regularization));

  for (const auto & particle : normalized_particles) {
    bool supported = false;
    for (const auto & component : mixture.components) {
      const Vector3 residual = pose_residual(particle.pose, component.mean);
      Matrix3 support_covariance = component.covariance;
      support_covariance[0] += config.covariance_regularization;
      support_covariance[4] += config.covariance_regularization;
      support_covariance[8] += config.covariance_regularization;
      if (mahalanobis_squared(residual, support_covariance) <=
          config.particle_support_mahalanobis_distance_squared)
      {
        supported = true;
        break;
      }
    }
    if (supported) {
      result.particle_mass_supported_by_gmm += particle.weight;
    }
  }

  result.particle_mass_supported_by_gmm =
    std::clamp(result.particle_mass_supported_by_gmm, 0.0, 1.0);
  return result;
}

}  // namespace hybrid_localization
