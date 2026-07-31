#include "hybrid_localization_core/recovery_sampling.hpp"

#include "hybrid_localization_core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hybrid_localization
{
namespace
{

constexpr double covariance_tolerance = 1e-12;
constexpr double mass_tolerance = 1e-12;

using Matrix3 = std::array<double, 9>;

[[nodiscard]] bool is_finite_pose(const Pose2d & pose) noexcept
{
  return
    std::isfinite(pose.x) &&
    std::isfinite(pose.y) &&
    std::isfinite(pose.yaw);
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

  if (
    covariance[0] < -covariance_tolerance ||
    covariance[4] < -covariance_tolerance ||
    covariance[8] < -covariance_tolerance)
  {
    throw std::invalid_argument(
            "Gaussian covariance diagonal must be nonnegative");
  }
}

[[nodiscard]] Matrix3 cholesky_factor(
  const Matrix3 & covariance,
  const double inflation_factor)
{
  Matrix3 lower{};

  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column <= row; ++column) {
      double value =
        covariance[row * 3U + column] * inflation_factor;

      for (std::size_t k = 0U; k < column; ++k) {
        value -=
          lower[row * 3U + k] *
          lower[column * 3U + k];
      }

      if (row == column) {
        if (value < -covariance_tolerance) {
          throw std::invalid_argument(
                  "Gaussian covariance must be positive semidefinite");
        }

        lower[row * 3U + column] =
          std::sqrt(std::max(0.0, value));
      } else {
        const double diagonal = lower[column * 3U + column];

        if (diagonal > covariance_tolerance) {
          lower[row * 3U + column] = value / diagonal;
        } else if (std::abs(value) > covariance_tolerance) {
          throw std::invalid_argument(
                  "Gaussian covariance must be positive semidefinite");
        }
      }
    }
  }

  return lower;
}

[[nodiscard]] std::vector<std::size_t> allocate_samples(
  const std::vector<double> & normalized_weights,
  const RecoverySamplingConfig & config)
{
  const std::size_t component_count = normalized_weights.size();

  if (
    config.minimum_samples_per_component > 0U &&
    component_count >
    config.particle_count / config.minimum_samples_per_component)
  {
    throw std::invalid_argument(
            "particle_count is too small for minimum_samples_per_component");
  }

  std::vector<std::size_t> allocations(
    component_count,
    config.minimum_samples_per_component);

  const std::size_t reserved_count =
    component_count * config.minimum_samples_per_component;
  const std::size_t remaining_count =
    config.particle_count - reserved_count;

  struct Remainder
  {
    std::size_t component_index{0U};
    double value{0.0};
  };

  std::vector<Remainder> remainders;
  remainders.reserve(component_count);

  std::size_t allocated_count = reserved_count;

  for (std::size_t index = 0U; index < component_count; ++index) {
    const double expected =
      static_cast<double>(remaining_count) * normalized_weights[index];
    const auto base = static_cast<std::size_t>(std::floor(expected));

    allocations[index] += base;
    allocated_count += base;
    remainders.push_back({index, expected - static_cast<double>(base)});
  }

  std::stable_sort(
    remainders.begin(),
    remainders.end(),
    [](const Remainder & lhs, const Remainder & rhs)
    {
      return lhs.value > rhs.value;
    });

  const std::size_t unallocated_count =
    config.particle_count - allocated_count;

  for (std::size_t index = 0U; index < unallocated_count; ++index) {
    ++allocations[remainders[index].component_index];
  }

  return allocations;
}

}  // namespace

std::vector<WeightedParticle> sample_recovery_particles(
  const GaussianMixture & mixture,
  const RecoverySamplingConfig & config)
{
  if (config.particle_count == 0U) {
    throw std::invalid_argument("Recovery particle count must be greater than zero");
  }

  if (
    !std::isfinite(config.covariance_inflation_factor) ||
    config.covariance_inflation_factor <= 0.0)
  {
    throw std::invalid_argument(
            "Covariance inflation factor must be finite and greater than zero");
  }

  if (mixture.components.empty()) {
    throw std::invalid_argument(
            "Cannot sample local recovery particles from an empty mixture");
  }

  if (
    !std::isfinite(mixture.discarded_weight) ||
    mixture.discarded_weight < 0.0 ||
    mixture.discarded_weight > 1.0 + mass_tolerance)
  {
    throw std::invalid_argument("Gaussian mixture discarded weight is invalid");
  }

  std::vector<double> normalized_weights;
  normalized_weights.reserve(mixture.components.size());

  std::vector<Matrix3> cholesky_factors;
  cholesky_factors.reserve(mixture.components.size());

  double represented_weight = 0.0;

  for (const GaussianComponent & component : mixture.components) {
    if (!is_finite_pose(component.mean)) {
      throw std::invalid_argument("Gaussian component mean must be finite");
    }

    if (!std::isfinite(component.weight) || component.weight <= 0.0) {
      throw std::invalid_argument(
              "Gaussian component weight must be finite and greater than zero");
    }

    validate_covariance(component.covariance);

    represented_weight += component.weight;
    normalized_weights.push_back(component.weight);
    cholesky_factors.push_back(
      cholesky_factor(
        component.covariance,
        config.covariance_inflation_factor));
  }

  if (!std::isfinite(represented_weight) || represented_weight <= 0.0) {
    throw std::invalid_argument("Gaussian mixture represented weight is invalid");
  }

  if (
    std::abs(
      represented_weight + mixture.discarded_weight - 1.0) > mass_tolerance)
  {
    throw std::invalid_argument(
            "Gaussian mixture probability mass is inconsistent");
  }

  for (double & weight : normalized_weights) {
    weight /= represented_weight;
  }

  const std::vector<std::size_t> allocations =
    allocate_samples(normalized_weights, config);

  std::mt19937_64 generator(config.random_seed);
  std::normal_distribution<double> standard_normal(0.0, 1.0);

  std::vector<WeightedParticle> particles;
  particles.reserve(config.particle_count);

  const double particle_weight =
    1.0 / static_cast<double>(config.particle_count);

  for (std::size_t component_index = 0U;
       component_index < mixture.components.size();
       ++component_index)
  {
    const GaussianComponent & component = mixture.components[component_index];
    const Matrix3 & lower = cholesky_factors[component_index];

    for (std::size_t sample = 0U;
         sample < allocations[component_index];
         ++sample)
    {
      const std::array<double, 3> normal{
        standard_normal(generator),
        standard_normal(generator),
        standard_normal(generator)};

      const double delta_x = lower[0] * normal[0];
      const double delta_y =
        lower[3] * normal[0] +
        lower[4] * normal[1];
      const double delta_yaw =
        lower[6] * normal[0] +
        lower[7] * normal[1] +
        lower[8] * normal[2];

      particles.push_back({
        {
          component.mean.x + delta_x,
          component.mean.y + delta_y,
          normalize_angle(component.mean.yaw + delta_yaw)
        },
        particle_weight
      });
    }
  }

  if (particles.size() != config.particle_count) {
    throw std::runtime_error(
            "Recovery sample allocation did not produce the requested count");
  }

  return particles;
}

}  // namespace hybrid_localization