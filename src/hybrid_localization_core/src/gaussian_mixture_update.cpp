#include "hybrid_localization_core/gaussian_mixture_update.hpp"

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

using Matrix3 = std::array<double, 9>;
using Vector3 = std::array<double, 3>;

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
  for (const double value : covariance) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string(name) + " must contain only finite values");
    }
  }

  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = row + 1U; column < 3U; ++column) {
      if (std::abs(
          covariance[index(row, column)] -
          covariance[index(column, row)]) > symmetry_tolerance)
      {
        throw std::invalid_argument(std::string(name) + " must be symmetric");
      }
    }
  }

  const double a = covariance[0U];
  const double b = covariance[1U];
  const double c = covariance[2U];
  const double d = covariance[4U];
  const double e = covariance[5U];
  const double f = covariance[8U];

  if (a < -psd_tolerance || d < -psd_tolerance || f < -psd_tolerance ||
    a * d - b * b < -psd_tolerance ||
    a * f - c * c < -psd_tolerance ||
    d * f - e * e < -psd_tolerance)
  {
    throw std::invalid_argument(std::string(name) + " must be positive semidefinite");
  }

  const double determinant =
    a * (d * f - e * e) -
    b * (b * f - c * e) +
    c * (b * e - c * d);

  if (determinant < -psd_tolerance) {
    throw std::invalid_argument(std::string(name) + " must be positive semidefinite");
  }
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
  Matrix3 result{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      result[index(row, column)] = matrix[index(column, row)];
    }
  }
  return result;
}

[[nodiscard]] Matrix3 multiply(const Matrix3 & lhs, const Matrix3 & rhs)
{
  Matrix3 result{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      for (std::size_t inner = 0U; inner < 3U; ++inner) {
        result[index(row, column)] +=
          lhs[index(row, inner)] * rhs[index(inner, column)];
      }
    }
  }
  return result;
}

[[nodiscard]] Vector3 multiply(const Matrix3 & matrix, const Vector3 & vector)
{
  Vector3 result{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      result[row] += matrix[index(row, column)] * vector[column];
    }
  }
  return result;
}

[[nodiscard]] Matrix3 add(const Matrix3 & lhs, const Matrix3 & rhs)
{
  Matrix3 result{};
  for (std::size_t i = 0U; i < result.size(); ++i) {
    result[i] = lhs[i] + rhs[i];
  }
  return result;
}

[[nodiscard]] Matrix3 subtract(const Matrix3 & lhs, const Matrix3 & rhs)
{
  Matrix3 result{};
  for (std::size_t i = 0U; i < result.size(); ++i) {
    result[i] = lhs[i] - rhs[i];
  }
  return result;
}

[[nodiscard]] Matrix3 identity()
{
  return Matrix3{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

void symmetrize(Matrix3 & covariance)
{
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = row + 1U; column < 3U; ++column) {
      const double average = 0.5 * (
        covariance[index(row, column)] + covariance[index(column, row)]);
      covariance[index(row, column)] = average;
      covariance[index(column, row)] = average;
    }
  }
}

[[nodiscard]] double determinant(const Matrix3 & matrix)
{
  return
    matrix[0U] * (matrix[4U] * matrix[8U] - matrix[5U] * matrix[7U]) -
    matrix[1U] * (matrix[3U] * matrix[8U] - matrix[5U] * matrix[6U]) +
    matrix[2U] * (matrix[3U] * matrix[7U] - matrix[4U] * matrix[6U]);
}

[[nodiscard]] Matrix3 inverse(const Matrix3 & matrix)
{
  const double det = determinant(matrix);
  if (!std::isfinite(det) || det <= 0.0) {
    throw std::invalid_argument("Innovation covariance must be positive definite");
  }

  Matrix3 result{};
  result[0U] = matrix[4U] * matrix[8U] - matrix[5U] * matrix[7U];
  result[1U] = matrix[2U] * matrix[7U] - matrix[1U] * matrix[8U];
  result[2U] = matrix[1U] * matrix[5U] - matrix[2U] * matrix[4U];
  result[3U] = matrix[5U] * matrix[6U] - matrix[3U] * matrix[8U];
  result[4U] = matrix[0U] * matrix[8U] - matrix[2U] * matrix[6U];
  result[5U] = matrix[2U] * matrix[3U] - matrix[0U] * matrix[5U];
  result[6U] = matrix[3U] * matrix[7U] - matrix[4U] * matrix[6U];
  result[7U] = matrix[1U] * matrix[6U] - matrix[0U] * matrix[7U];
  result[8U] = matrix[0U] * matrix[4U] - matrix[1U] * matrix[3U];

  for (double & value : result) {
    value /= det;
  }
  return result;
}

[[nodiscard]] double quadratic_form(
  const Vector3 & vector,
  const Matrix3 & matrix)
{
  const Vector3 transformed = multiply(matrix, vector);
  double result = 0.0;
  for (std::size_t i = 0U; i < vector.size(); ++i) {
    result += vector[i] * transformed[i];
  }
  return result;
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