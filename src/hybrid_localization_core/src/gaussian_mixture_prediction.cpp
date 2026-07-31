#include "hybrid_localization_core/gaussian_mixture_prediction.hpp"

#include "hybrid_localization_core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace hybrid_localization
{
namespace
{

using Matrix3 = std::array<double, 9>;

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

  // Positive-semidefinite check using principal minors for a symmetric 3x3 matrix.
  const double a = covariance[0U];
  const double b = covariance[1U];
  const double c = covariance[2U];
  const double d = covariance[4U];
  const double e = covariance[5U];
  const double f = covariance[8U];

  if (a < -psd_tolerance || d < -psd_tolerance || f < -psd_tolerance) {
    throw std::invalid_argument(std::string(name) + " must be positive semidefinite");
  }

  if (a * d - b * b < -psd_tolerance ||
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

[[nodiscard]] Matrix3 multiply(
  const Matrix3 & lhs,
  const Matrix3 & rhs)
{
  Matrix3 result{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      double value = 0.0;
      for (std::size_t inner = 0U; inner < 3U; ++inner) {
        value += lhs[index(row, inner)] * rhs[index(inner, column)];
      }
      result[index(row, column)] = value;
    }
  }
  return result;
}

[[nodiscard]] Matrix3 add(
  const Matrix3 & lhs,
  const Matrix3 & rhs)
{
  Matrix3 result{};
  for (std::size_t i = 0U; i < result.size(); ++i) {
    result[i] = lhs[i] + rhs[i];
  }
  return result;
}

void symmetrize(Matrix3 & covariance)
{
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = row + 1U; column < 3U; ++column) {
      const double average = 0.5 * (
        covariance[index(row, column)] +
        covariance[index(column, row)]);
      covariance[index(row, column)] = average;
      covariance[index(column, row)] = average;
    }
  }
}

void validate_config(const GaussianPredictionConfig & config)
{
  validate_nonnegative(config.minimum_position_variance, "minimum_position_variance");
  validate_nonnegative(config.minimum_yaw_variance, "minimum_yaw_variance");
  validate_nonnegative(
    config.covariance_symmetry_tolerance,
    "covariance_symmetry_tolerance");
  validate_nonnegative(config.covariance_psd_tolerance, "covariance_psd_tolerance");
}

}  // namespace

GaussianComponent predict_gaussian_component(
  const GaussianComponent & component,
  const GaussianMotionIncrement & motion,
  const GaussianPredictionConfig & config)
{
  validate_config(config);
  validate_pose(component.mean, "Gaussian component mean");
  validate_pose(motion.mean, "Gaussian motion increment mean");

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
    motion.covariance,
    config.covariance_symmetry_tolerance,
    config.covariance_psd_tolerance,
    "Gaussian motion covariance");

  const double cosine = std::cos(component.mean.yaw);
  const double sine = std::sin(component.mean.yaw);

  GaussianComponent predicted = component;
  predicted.mean.x =
    component.mean.x + cosine * motion.mean.x - sine * motion.mean.y;
  predicted.mean.y =
    component.mean.y + sine * motion.mean.x + cosine * motion.mean.y;
  predicted.mean.yaw = normalize_angle(component.mean.yaw + motion.mean.yaw);

  const double yaw_x_derivative =
    -sine * motion.mean.x - cosine * motion.mean.y;
  const double yaw_y_derivative =
    cosine * motion.mean.x - sine * motion.mean.y;

  const Matrix3 state_jacobian{
    1.0, 0.0, yaw_x_derivative,
    0.0, 1.0, yaw_y_derivative,
    0.0, 0.0, 1.0};

  const Matrix3 motion_jacobian{
    cosine, -sine, 0.0,
    sine, cosine, 0.0,
    0.0, 0.0, 1.0};

  const Matrix3 propagated_state = multiply(
    multiply(state_jacobian, component.covariance),
    transpose(state_jacobian));
  const Matrix3 propagated_motion = multiply(
    multiply(motion_jacobian, motion.covariance),
    transpose(motion_jacobian));

  predicted.covariance = add(propagated_state, propagated_motion);
  symmetrize(predicted.covariance);

  predicted.covariance[0U] = std::max(
    predicted.covariance[0U],
    config.minimum_position_variance);
  predicted.covariance[4U] = std::max(
    predicted.covariance[4U],
    config.minimum_position_variance);
  predicted.covariance[8U] = std::max(
    predicted.covariance[8U],
    config.minimum_yaw_variance);

  validate_covariance(
    predicted.covariance,
    config.covariance_symmetry_tolerance,
    config.covariance_psd_tolerance,
    "Predicted Gaussian covariance");

  return predicted;
}

GaussianMixture predict_gaussian_mixture(
  const GaussianMixture & mixture,
  const GaussianMotionIncrement & motion,
  const GaussianPredictionConfig & config)
{
  if (!std::isfinite(mixture.discarded_weight) ||
    mixture.discarded_weight < 0.0 ||
    mixture.discarded_weight > 1.0)
  {
    throw std::invalid_argument("discarded_weight must lie in [0, 1]");
  }

  GaussianMixture predicted;
  predicted.discarded_weight = mixture.discarded_weight;
  predicted.components.reserve(mixture.components.size());

  double represented_weight = 0.0;
  for (const auto & component : mixture.components) {
    represented_weight += component.weight;
    predicted.components.push_back(
      predict_gaussian_component(component, motion, config));
  }

  constexpr double mass_tolerance = 1e-12;
  if (!std::isfinite(represented_weight) ||
    std::abs(represented_weight + mixture.discarded_weight - 1.0) > mass_tolerance)
  {
    throw std::invalid_argument("Gaussian mixture probability mass must sum to one");
  }

  return predicted;
}

}  // namespace hybrid_localization