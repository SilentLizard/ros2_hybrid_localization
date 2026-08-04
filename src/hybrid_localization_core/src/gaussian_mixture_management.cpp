#include "hybrid_localization_core/gaussian_mixture_management.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace hybrid_localization
{
namespace
{

void validate_config(const GaussianMixtureManagementConfig & config)
{
  if (!std::isfinite(config.minimum_component_weight) ||
    config.minimum_component_weight < 0.0 ||
    config.minimum_component_weight > 1.0)
  {
    throw std::invalid_argument(
            "Minimum component weight must be finite and lie in [0, 1]");
  }

  if (!std::isfinite(config.mass_tolerance) || config.mass_tolerance < 0.0) {
    throw std::invalid_argument(
            "Mass tolerance must be finite and nonnegative");
  }
}

void validate_mixture_weights(const GaussianMixture & mixture)
{
  if (!std::isfinite(mixture.discarded_weight) ||
    mixture.discarded_weight < 0.0)
  {
    throw std::invalid_argument(
            "Discarded mixture weight must be finite and nonnegative");
  }

  for (const auto & component : mixture.components) {
    if (!std::isfinite(component.weight) || component.weight < 0.0) {
      throw std::invalid_argument(
              "Gaussian component weight must be finite and nonnegative");
    }
  }
}

}  // namespace

GaussianMixtureManagementResult manage_gaussian_mixture(
  const GaussianMixture & mixture,
  const GaussianMixtureManagementConfig & config)
{
  validate_config(config);
  validate_mixture_weights(mixture);

  const double component_mass = std::accumulate(
    mixture.components.begin(),
    mixture.components.end(),
    0.0,
    [](const double total, const GaussianComponent & component) {
      return total + component.weight;
    });

  const double total_mass = component_mass + mixture.discarded_weight;
  if (!std::isfinite(total_mass) || total_mass <= 0.0) {
    throw std::invalid_argument(
            "Gaussian mixture must contain positive finite probability mass");
  }

  GaussianMixtureManagementResult result;
  result.normalization_scale = 1.0 / total_mass;
  result.mixture.discarded_weight =
    mixture.discarded_weight * result.normalization_scale;
  result.mixture.components = mixture.components;

  for (auto & component : result.mixture.components) {
    component.weight *= result.normalization_scale;
  }

  std::stable_sort(
    result.mixture.components.begin(),
    result.mixture.components.end(),
    [](const GaussianComponent & lhs, const GaussianComponent & rhs) {
      return lhs.weight > rhs.weight;
    });

  std::vector<GaussianComponent> retained;
  retained.reserve(std::min(
      result.mixture.components.size(),
      config.maximum_component_count));

  for (auto & component : result.mixture.components) {
    const bool below_threshold =
      component.weight < config.minimum_component_weight;
    const bool exceeds_count =
      retained.size() >= config.maximum_component_count;

    if (below_threshold || exceeds_count) {
      result.pruned_weight += component.weight;
      if (has_hypothesis_id(component.provenance)) {
        result.pruned_hypothesis_ids.push_back(component.provenance.id);
      }
      ++result.pruned_component_count;
      continue;
    }

    retained.push_back(std::move(component));
  }

  result.mixture.components = std::move(retained);
  result.mixture.discarded_weight += result.pruned_weight;

  const double retained_mass = std::accumulate(
    result.mixture.components.begin(),
    result.mixture.components.end(),
    0.0,
    [](const double total, const GaussianComponent & component) {
      return total + component.weight;
    });

  const double normalized_mass =
    retained_mass + result.mixture.discarded_weight;

  if (!std::isfinite(normalized_mass) ||
    std::abs(normalized_mass - 1.0) > config.mass_tolerance)
  {
    throw std::runtime_error(
            "Managed Gaussian mixture probability mass is inconsistent");
  }

  return result;
}

}  // namespace hybrid_localization