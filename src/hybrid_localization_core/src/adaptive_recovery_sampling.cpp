#include "hybrid_localization_core/adaptive_recovery_sampling.hpp"

#include "hybrid_localization_core/global_particle_sampling.hpp"
#include "hybrid_localization_core/recovery_sampling.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace hybrid_localization
{
namespace
{

constexpr double mass_tolerance = 1e-12;

void validate_unit_interval(const double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
    throw std::invalid_argument(std::string(name) + " must lie in [0, 1]");
  }
}

void validate_nonnegative(const double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
  }
}

struct MixtureMass
{
  double represented{0.0};
  double total{0.0};
};

[[nodiscard]] MixtureMass validate_mixture_mass(const GaussianMixture & mixture)
{
  validate_unit_interval(mixture.discarded_weight, "discarded_weight");

  MixtureMass mass;
  for (const auto & component : mixture.components) {
    if (!std::isfinite(component.weight) || component.weight < 0.0) {
      throw std::invalid_argument(
              "Gaussian component weights must be finite and nonnegative");
    }
    mass.represented += component.weight;
  }

  mass.total = mass.represented + mixture.discarded_weight;
  if (!std::isfinite(mass.total) || std::abs(mass.total - 1.0) > mass_tolerance) {
    throw std::invalid_argument("Gaussian mixture probability mass must sum to one");
  }

  return mass;
}

[[nodiscard]] std::uint64_t derive_seed(
  const std::uint64_t seed,
  const std::uint64_t stream)
{
  // SplitMix64-style deterministic stream separation.
  std::uint64_t value = seed + 0x9E3779B97F4A7C15ULL * (stream + 1U);
  value = (value ^ (value >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  value = (value ^ (value >> 27U)) * 0x94D049BB133111EBULL;
  return value ^ (value >> 31U);
}

}  // namespace

double normalized_mixture_entropy(const GaussianMixture & mixture)
{
  const auto mass = validate_mixture_mass(mixture);

  std::size_t positive_components = 0U;
  for (const auto & component : mixture.components) {
    if (component.weight > 0.0) {
      ++positive_components;
    }
  }

  if (positive_components <= 1U || mass.represented <= 0.0) {
    return 0.0;
  }

  double entropy = 0.0;
  for (const auto & component : mixture.components) {
    if (component.weight <= 0.0) {
      continue;
    }

    const double probability = component.weight / mass.represented;
    entropy -= probability * std::log(probability);
  }

  const double maximum_entropy =
    std::log(static_cast<double>(positive_components));
  return std::clamp(entropy / maximum_entropy, 0.0, 1.0);
}

AdaptiveRecoverySamplingResult sample_adaptive_recovery_particles(
  const GaussianMixture & mixture,
  const OccupancyGridView & grid,
  const RecoveryAllocationSignals & signals,
  const AdaptiveRecoverySamplingConfig & config)
{
  if (config.particle_count == 0U) {
    throw std::invalid_argument("Recovery particle count must be greater than zero");
  }

  validate_unit_interval(config.base_global_fraction, "base_global_fraction");
  validate_unit_interval(config.minimum_global_fraction, "minimum_global_fraction");
  validate_unit_interval(config.maximum_global_fraction, "maximum_global_fraction");
  if (config.minimum_global_fraction > config.maximum_global_fraction) {
    throw std::invalid_argument(
            "minimum_global_fraction must not exceed maximum_global_fraction");
  }

  validate_nonnegative(config.discarded_weight_gain, "discarded_weight_gain");
  validate_nonnegative(config.mixture_entropy_gain, "mixture_entropy_gain");
  validate_nonnegative(config.failure_score_gain, "failure_score_gain");
  validate_unit_interval(signals.failure_score, "failure_score");

  const auto mass = validate_mixture_mass(mixture);
  const double entropy = normalized_mixture_entropy(mixture);

  double requested_global_fraction =
    config.base_global_fraction +
    config.discarded_weight_gain * mixture.discarded_weight +
    config.mixture_entropy_gain * entropy +
    config.failure_score_gain * signals.failure_score;

  requested_global_fraction = std::clamp(
    requested_global_fraction,
    config.minimum_global_fraction,
    config.maximum_global_fraction);

  // With no represented component, local sampling is impossible.
  if (mass.represented <= 0.0) {
    requested_global_fraction = 1.0;
  }

  const double requested_global_count =
    requested_global_fraction * static_cast<double>(config.particle_count);
  const auto global_count = static_cast<std::size_t>(
    std::llround(requested_global_count));
  const std::size_t local_count = config.particle_count - global_count;

  if (local_count > 0U && mass.represented <= 0.0) {
    throw std::invalid_argument(
            "Local recovery particles require represented Gaussian components");
  }

  AdaptiveRecoverySamplingResult result;
  result.local_particle_count = local_count;
  result.global_particle_count = global_count;
  result.global_fraction =
    static_cast<double>(global_count) /
    static_cast<double>(config.particle_count);
  result.normalized_mixture_entropy = entropy;
  result.particles.reserve(config.particle_count);

  if (local_count > 0U) {
    RecoverySamplingConfig local_config;
    local_config.particle_count = local_count;
    local_config.covariance_inflation_factor =
      config.covariance_inflation_factor;
    local_config.random_seed = derive_seed(config.random_seed, 0U);
    local_config.minimum_samples_per_component =
      config.minimum_samples_per_component;

    auto local_particles = sample_recovery_particles(mixture, local_config);
    result.particles.insert(
      result.particles.end(),
      local_particles.begin(),
      local_particles.end());
  }

  if (global_count > 0U) {
    GlobalParticleSamplingConfig global_config;
    global_config.particle_count = global_count;
    global_config.maximum_occupancy = config.maximum_occupancy;
    global_config.unknown_cell_policy = config.unknown_cell_policy;
    global_config.random_seed = derive_seed(config.random_seed, 1U);

    auto global_particles = sample_global_particles(grid, global_config);
    result.particles.insert(
      result.particles.end(),
      global_particles.begin(),
      global_particles.end());
  }

  if (result.particles.size() != config.particle_count) {
    throw std::runtime_error("Adaptive recovery produced an incorrect particle count");
  }

  std::mt19937_64 shuffle_generator(derive_seed(config.random_seed, 2U));
  std::shuffle(
    result.particles.begin(),
    result.particles.end(),
    shuffle_generator);

  const double normalized_weight =
    1.0 / static_cast<double>(config.particle_count);
  for (auto & particle : result.particles) {
    particle.weight = normalized_weight;
  }

  return result;
}

}  // namespace hybrid_localization