#include "hybrid_localization_core/gaussian_mixture_update.hpp"

#include "hybrid_localization_core/detail/matrix3.hpp"
#include "hybrid_localization_core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace hybrid_localization
{
namespace
{

using Matrix3 = detail::Matrix3Storage;
using Vector3 = detail::Vector3Storage;

[[nodiscard]] constexpr std::size_t index(
  const std::size_t row,
  const std::size_t column) noexcept
{
  return row * 3U + column;
}

void validate_nonnegative(const double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
  }
}

void validate_pose(const Pose2d & pose, const char * name)
{
  if (!std::isfinite(pose.x) ||
    !std::isfinite(pose.y) ||
    !std::isfinite(pose.yaw))
  {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void validate_covariance(
  const Matrix3 & covariance,
  const double symmetry_tolerance,
  const double psd_tolerance,
  const char * name)
{
  detail::validate_covariance(covariance, symmetry_tolerance, psd_tolerance, name);
}

void validate_config(const GaussianUpdateConfig & config)
{
  if (std::isnan(config.maximum_mahalanobis_distance_squared) ||
    config.maximum_mahalanobis_distance_squared < 0.0)
  {
    throw std::invalid_argument(
            "maximum_mahalanobis_distance_squared must be nonnegative");
  }
  validate_nonnegative(config.likelihood_floor, "likelihood_floor");
  validate_nonnegative(
    config.innovation_covariance_regularization,
    "innovation_covariance_regularization");
  validate_nonnegative(
    config.covariance_symmetry_tolerance,
    "covariance_symmetry_tolerance");
  validate_nonnegative(config.covariance_psd_tolerance, "covariance_psd_tolerance");
}

[[nodiscard]] Matrix3 transpose(const Matrix3 & matrix)
{
  return detail::transpose(matrix);
}

[[nodiscard]] Matrix3 multiply(const Matrix3 & lhs, const Matrix3 & rhs)
{
  return detail::multiply(lhs, rhs);
}

[[nodiscard]] Vector3 multiply(const Matrix3 & matrix, const Vector3 & vector)
{
  return detail::multiply(matrix, vector);
}

[[nodiscard]] Matrix3 add(const Matrix3 & lhs, const Matrix3 & rhs)
{
  return detail::add(lhs, rhs);
}

[[nodiscard]] Matrix3 subtract(const Matrix3 & lhs, const Matrix3 & rhs)
{
  return detail::subtract(lhs, rhs);
}

[[nodiscard]] Matrix3 identity()
{
  return detail::identity();
}

void symmetrize(Matrix3 & covariance)
{
  detail::symmetrize(covariance);
}

[[nodiscard]] double determinant(const Matrix3 & matrix)
{
  return detail::determinant(matrix);
}

[[nodiscard]] Matrix3 inverse(const Matrix3 & matrix)
{
  return detail::inverse_positive_definite(matrix, "Innovation covariance");
}

[[nodiscard]] double quadratic_form(
  const Vector3 & vector,
  const Matrix3 & matrix)
{
  return detail::quadratic_form(vector, matrix);
}

[[nodiscard]] double gaussian_likelihood(
  const double mahalanobis_squared,
  const double innovation_determinant)
{
  constexpr double dimension = 3.0;
  const double normalization = std::pow(2.0 * std::numbers::pi, dimension / 2.0) *
    std::sqrt(innovation_determinant);
  return std::exp(-0.5 * mahalanobis_squared) / normalization;
}

}  // namespace

GaussianComponentUpdate update_gaussian_component(
  const GaussianComponent & component,
  const GaussianPoseObservation & observation,
  const GaussianUpdateConfig & config)
{
  validate_config(config);
  validate_pose(component.mean, "Gaussian component mean");
  validate_pose(observation.mean, "Gaussian observation mean");

  if (!std::isfinite(component.weight) || component.weight < 0.0) {
    throw std::invalid_argument(
            "Gaussian component weight must be finite and nonnegative");
  }

  validate_covariance(
    component.covariance,
    config.covariance_symmetry_tolerance,
    config.covariance_psd_tolerance,
    "Gaussian component covariance");
  validate_covariance(
    observation.covariance,
    config.covariance_symmetry_tolerance,
    config.covariance_psd_tolerance,
    "Gaussian observation covariance");

  GaussianComponentUpdate result;
  result.component = component;
  result.innovation = Pose2d{
    observation.mean.x - component.mean.x,
    observation.mean.y - component.mean.y,
    angle_difference(observation.mean.yaw, component.mean.yaw)};

  Matrix3 innovation_covariance = add(component.covariance, observation.covariance);
  innovation_covariance[0U] += config.innovation_covariance_regularization;
  innovation_covariance[4U] += config.innovation_covariance_regularization;
  innovation_covariance[8U] += config.innovation_covariance_regularization;
  symmetrize(innovation_covariance);

  const double innovation_determinant = determinant(innovation_covariance);
  const Matrix3 innovation_inverse = inverse(innovation_covariance);
  const Vector3 innovation_vector{
    result.innovation.x,
    result.innovation.y,
    result.innovation.yaw};

  result.mahalanobis_distance_squared = std::max(
    0.0,
    quadratic_form(innovation_vector, innovation_inverse));

  const double raw_likelihood = gaussian_likelihood(
    result.mahalanobis_distance_squared,
    innovation_determinant);
  result.likelihood = std::max(raw_likelihood, config.likelihood_floor);

  result.accepted =
    result.mahalanobis_distance_squared <=
    config.maximum_mahalanobis_distance_squared;

  if (!result.accepted) {
    result.likelihood = config.likelihood_floor;
    return result;
  }

  const Matrix3 kalman_gain = multiply(component.covariance, innovation_inverse);
  const Vector3 correction = multiply(kalman_gain, innovation_vector);

  result.component.mean.x += correction[0U];
  result.component.mean.y += correction[1U];
  result.component.mean.yaw = normalize_angle(
    result.component.mean.yaw + correction[2U]);

  // Joseph stabilized covariance update for H = I:
  // P' = (I-K) P (I-K)^T + K R K^T
  const Matrix3 identity_matrix = identity();
  const Matrix3 residual_gain = subtract(identity_matrix, kalman_gain);
  result.component.covariance = add(
    multiply(
      multiply(residual_gain, component.covariance),
      transpose(residual_gain)),
    multiply(
      multiply(kalman_gain, observation.covariance),
      transpose(kalman_gain)));
  symmetrize(result.component.covariance);

  validate_covariance(
    result.component.covariance,
    config.covariance_symmetry_tolerance,
    config.covariance_psd_tolerance,
    "Updated Gaussian covariance");

  return result;
}

GaussianMixtureUpdate update_gaussian_mixture(
  const GaussianMixture & mixture,
  const GaussianPoseObservation & observation,
  const GaussianUpdateConfig & config)
{
  if (!std::isfinite(mixture.discarded_weight) ||
    mixture.discarded_weight < 0.0 ||
    mixture.discarded_weight > 1.0)
  {
    throw std::invalid_argument("discarded_weight must lie in [0, 1]");
  }

  const double represented_mass = 1.0 - mixture.discarded_weight;
  if (mixture.components.empty()) {
    if (represented_mass > 1e-12) {
      throw std::invalid_argument(
              "Nonzero represented mass requires at least one component");
    }
    return GaussianMixtureUpdate{mixture, {}, 0.0};
  }

  GaussianMixtureUpdate result;
  result.mixture.discarded_weight = mixture.discarded_weight;
  result.mixture.components.reserve(mixture.components.size());
  result.component_updates.reserve(mixture.components.size());

  double input_weight_sum = 0.0;
  double evidence = 0.0;
  for (const auto & component : mixture.components) {
    input_weight_sum += component.weight;
    auto update = update_gaussian_component(component, observation, config);
    evidence += component.weight * update.likelihood;
    result.component_updates.push_back(update);
  }

  constexpr double mass_tolerance = 1e-12;
  if (!std::isfinite(input_weight_sum) ||
    std::abs(input_weight_sum - represented_mass) > mass_tolerance)
  {
    throw std::invalid_argument("Gaussian mixture probability mass must sum to one");
  }

  if (!std::isfinite(evidence) || evidence <= 0.0) {
    throw std::runtime_error("Gaussian mixture measurement evidence is not positive");
  }

  result.normalization_evidence = evidence;
  for (auto & update : result.component_updates) {
    update.component.weight = represented_mass *
      (update.component.weight * update.likelihood / evidence);
    result.mixture.components.push_back(update.component);
  }

  return result;
}

}  // namespace hybrid_localization