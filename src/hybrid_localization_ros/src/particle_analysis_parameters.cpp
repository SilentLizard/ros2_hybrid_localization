#include "hybrid_localization_ros/particle_analysis_parameters.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace hybrid_localization_ros
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = description;
  return result;
}

template<typename T>
T declare_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const T & value,
  const std::string & description)
{
  return node.declare_parameter<T>(name, value, descriptor(description));
}

std::size_t checked_count(const std::int64_t value, const std::string & name)
{
  if (value < 0) {
    throw std::invalid_argument(name + " must be non-negative");
  }

  const auto unsigned_value = static_cast<std::uint64_t>(value);
  if (unsigned_value > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    throw std::invalid_argument(name + " exceeds size_t range");
  }
  return static_cast<std::size_t>(unsigned_value);
}

std::size_t parameter_count(const rclcpp::Parameter & parameter)
{
  return checked_count(parameter.as_int(), parameter.get_name());
}

void validate(const ParticleAnalysisProcessorConfig & config)
{
  hybrid_localization::validate_particle_clustering_config(config.clustering);
  hybrid_localization::validate_localization_evidence_policy_config(config.evidence_policy);
}

}  // namespace

ParticleAnalysisProcessorConfig declare_particle_analysis_parameters(
  rclcpp::Node & node,
  const ParticleAnalysisProcessorConfig & defaults)
{
  ParticleAnalysisProcessorConfig config = defaults;

  config.clustering.position_scale = declare_parameter<double>(
    node, "particle_clustering.position_scale", config.clustering.position_scale,
    "Translation represented by one normalized clustering-distance unit [m].");
  config.clustering.yaw_scale = declare_parameter<double>(
    node, "particle_clustering.yaw_scale", config.clustering.yaw_scale,
    "Yaw represented by one normalized clustering-distance unit [rad].");
  config.clustering.epsilon = declare_parameter<double>(
    node, "particle_clustering.epsilon", config.clustering.epsilon,
    "Maximum normalized weighted-DBSCAN neighborhood distance.");
  config.clustering.minimum_neighbors = checked_count(
    declare_parameter<std::int64_t>(
      node, "particle_clustering.minimum_neighbors",
      static_cast<std::int64_t>(config.clustering.minimum_neighbors),
      "Minimum particle count in a core neighborhood, including the center particle."),
    "particle_clustering.minimum_neighbors");
  config.clustering.minimum_core_weight = declare_parameter<double>(
    node, "particle_clustering.minimum_core_weight", config.clustering.minimum_core_weight,
    "Minimum normalized probability mass in a core neighborhood.");
  config.clustering.minimum_cluster_weight = declare_parameter<double>(
    node, "particle_clustering.minimum_cluster_weight", config.clustering.minimum_cluster_weight,
    "Minimum normalized probability mass retained as a particle cluster.");

  auto & policy = config.evidence_policy;
  policy.particle_minimum_retained_weight = declare_parameter<double>(
    node, "health_policy.particle_minimum_retained_weight", policy.particle_minimum_retained_weight,
    "Minimum retained particle mass required for convergence evidence.");
  policy.particle_minimum_dominant_weight = declare_parameter<double>(
    node, "health_policy.particle_minimum_dominant_weight", policy.particle_minimum_dominant_weight,
    "Minimum dominant particle-cluster mass required for convergence evidence.");
  policy.particle_maximum_noise_weight = declare_parameter<double>(
    node, "health_policy.particle_maximum_noise_weight", policy.particle_maximum_noise_weight,
    "Maximum particle noise mass allowed for convergence evidence.");
  policy.particle_maximum_retained_clusters = checked_count(
    declare_parameter<std::int64_t>(
      node, "health_policy.particle_maximum_retained_clusters",
      static_cast<std::int64_t>(policy.particle_maximum_retained_clusters),
      "Maximum retained particle-cluster count allowed for convergence evidence."),
    "health_policy.particle_maximum_retained_clusters");
  policy.gmm_minimum_available_represented_weight = declare_parameter<double>(
    node, "health_policy.gmm_minimum_available_represented_weight",
    policy.gmm_minimum_available_represented_weight,
    "Minimum represented GMM mass required to consider a mixture available.");

  policy.good_minimum_represented_weight = declare_parameter<double>(
    node, "health_policy.good_minimum_represented_weight", policy.good_minimum_represented_weight,
    "Healthy-entry minimum represented GMM mass.");
  policy.good_minimum_dominant_weight = declare_parameter<double>(
    node, "health_policy.good_minimum_dominant_weight", policy.good_minimum_dominant_weight,
    "Healthy-entry minimum dominant-component mass.");
  policy.good_maximum_normalized_entropy = declare_parameter<double>(
    node, "health_policy.good_maximum_normalized_entropy", policy.good_maximum_normalized_entropy,
    "Healthy-entry maximum normalized mixture entropy.");
  policy.good_maximum_weighted_position_variance = declare_parameter<double>(
    node, "health_policy.good_maximum_weighted_position_variance",
    policy.good_maximum_weighted_position_variance,
    "Healthy-entry maximum weighted position variance [m^2].");
  policy.good_maximum_weighted_yaw_variance = declare_parameter<double>(
    node, "health_policy.good_maximum_weighted_yaw_variance",
    policy.good_maximum_weighted_yaw_variance,
    "Healthy-entry maximum weighted yaw variance [rad^2].");

  policy.bad_maximum_represented_weight = declare_parameter<double>(
    node, "health_policy.bad_maximum_represented_weight", policy.bad_maximum_represented_weight,
    "Bad-entry maximum represented GMM mass.");
  policy.bad_maximum_dominant_weight = declare_parameter<double>(
    node, "health_policy.bad_maximum_dominant_weight", policy.bad_maximum_dominant_weight,
    "Bad-entry maximum dominant-component mass.");
  policy.bad_minimum_normalized_entropy = declare_parameter<double>(
    node, "health_policy.bad_minimum_normalized_entropy", policy.bad_minimum_normalized_entropy,
    "Bad-entry minimum normalized mixture entropy.");
  policy.bad_minimum_weighted_position_variance = declare_parameter<double>(
    node, "health_policy.bad_minimum_weighted_position_variance",
    policy.bad_minimum_weighted_position_variance,
    "Bad-entry minimum weighted position variance [m^2].");
  policy.bad_minimum_weighted_yaw_variance = declare_parameter<double>(
    node, "health_policy.bad_minimum_weighted_yaw_variance",
    policy.bad_minimum_weighted_yaw_variance,
    "Bad-entry minimum weighted yaw variance [rad^2].");

  policy.ambiguity_entry_minimum_entropy = declare_parameter<double>(
    node, "health_policy.ambiguity_entry_minimum_entropy",
    policy.ambiguity_entry_minimum_entropy,
    "Minimum normalized entropy for ambiguity entry.");
  policy.ambiguity_exit_maximum_entropy = declare_parameter<double>(
    node, "health_policy.ambiguity_exit_maximum_entropy",
    policy.ambiguity_exit_maximum_entropy,
    "Maximum normalized entropy for ambiguity exit.");
  policy.ambiguity_entry_minimum_effective_component_count = declare_parameter<double>(
    node, "health_policy.ambiguity_entry_minimum_effective_component_count",
    policy.ambiguity_entry_minimum_effective_component_count,
    "Minimum effective component count for ambiguity entry.");
  policy.ambiguity_exit_maximum_effective_component_count = declare_parameter<double>(
    node, "health_policy.ambiguity_exit_maximum_effective_component_count",
    policy.ambiguity_exit_maximum_effective_component_count,
    "Maximum effective component count for ambiguity exit.");
  policy.ambiguity_entry_maximum_dominant_weight = declare_parameter<double>(
    node, "health_policy.ambiguity_entry_maximum_dominant_weight",
    policy.ambiguity_entry_maximum_dominant_weight,
    "Maximum dominant-component mass that can trigger ambiguity entry.");
  policy.ambiguity_exit_minimum_dominant_weight = declare_parameter<double>(
    node, "health_policy.ambiguity_exit_minimum_dominant_weight",
    policy.ambiguity_exit_minimum_dominant_weight,
    "Minimum dominant-component mass required for ambiguity exit.");
  policy.ambiguity_component_budget_fraction = declare_parameter<double>(
    node, "health_policy.ambiguity_component_budget_fraction",
    policy.ambiguity_component_budget_fraction,
    "Fraction of the configured component budget considered ambiguity pressure.");
  policy.maximum_component_count = checked_count(
    declare_parameter<std::int64_t>(
      node, "health_policy.maximum_component_count",
      static_cast<std::int64_t>(policy.maximum_component_count),
      "Component-count budget used by observation-mode ambiguity evidence."),
    "health_policy.maximum_component_count");

  validate(config);
  return config;
}

ParticleAnalysisProcessorConfig apply_particle_analysis_parameter_updates(
  const ParticleAnalysisProcessorConfig & current,
  const std::span<const rclcpp::Parameter> parameters)
{
  ParticleAnalysisProcessorConfig candidate = current;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "particle_clustering.position_scale") {
      candidate.clustering.position_scale = parameter.as_double();
    } else if (name == "particle_clustering.yaw_scale") {
      candidate.clustering.yaw_scale = parameter.as_double();
    } else if (name == "particle_clustering.epsilon") {
      candidate.clustering.epsilon = parameter.as_double();
    } else if (name == "particle_clustering.minimum_neighbors") {
      candidate.clustering.minimum_neighbors = parameter_count(parameter);
    } else if (name == "particle_clustering.minimum_core_weight") {
      candidate.clustering.minimum_core_weight = parameter.as_double();
    } else if (name == "particle_clustering.minimum_cluster_weight") {
      candidate.clustering.minimum_cluster_weight = parameter.as_double();
    } else if (name == "health_policy.particle_minimum_retained_weight") {
      candidate.evidence_policy.particle_minimum_retained_weight = parameter.as_double();
    } else if (name == "health_policy.particle_minimum_dominant_weight") {
      candidate.evidence_policy.particle_minimum_dominant_weight = parameter.as_double();
    } else if (name == "health_policy.particle_maximum_noise_weight") {
      candidate.evidence_policy.particle_maximum_noise_weight = parameter.as_double();
    } else if (name == "health_policy.particle_maximum_retained_clusters") {
      candidate.evidence_policy.particle_maximum_retained_clusters = parameter_count(parameter);
    } else if (name == "health_policy.gmm_minimum_available_represented_weight") {
      candidate.evidence_policy.gmm_minimum_available_represented_weight = parameter.as_double();
    } else if (name == "health_policy.good_minimum_represented_weight") {
      candidate.evidence_policy.good_minimum_represented_weight = parameter.as_double();
    } else if (name == "health_policy.good_minimum_dominant_weight") {
      candidate.evidence_policy.good_minimum_dominant_weight = parameter.as_double();
    } else if (name == "health_policy.good_maximum_normalized_entropy") {
      candidate.evidence_policy.good_maximum_normalized_entropy = parameter.as_double();
    } else if (name == "health_policy.good_maximum_weighted_position_variance") {
      candidate.evidence_policy.good_maximum_weighted_position_variance = parameter.as_double();
    } else if (name == "health_policy.good_maximum_weighted_yaw_variance") {
      candidate.evidence_policy.good_maximum_weighted_yaw_variance = parameter.as_double();
    } else if (name == "health_policy.bad_maximum_represented_weight") {
      candidate.evidence_policy.bad_maximum_represented_weight = parameter.as_double();
    } else if (name == "health_policy.bad_maximum_dominant_weight") {
      candidate.evidence_policy.bad_maximum_dominant_weight = parameter.as_double();
    } else if (name == "health_policy.bad_minimum_normalized_entropy") {
      candidate.evidence_policy.bad_minimum_normalized_entropy = parameter.as_double();
    } else if (name == "health_policy.bad_minimum_weighted_position_variance") {
      candidate.evidence_policy.bad_minimum_weighted_position_variance = parameter.as_double();
    } else if (name == "health_policy.bad_minimum_weighted_yaw_variance") {
      candidate.evidence_policy.bad_minimum_weighted_yaw_variance = parameter.as_double();
    } else if (name == "health_policy.ambiguity_entry_minimum_entropy") {
      candidate.evidence_policy.ambiguity_entry_minimum_entropy = parameter.as_double();
    } else if (name == "health_policy.ambiguity_exit_maximum_entropy") {
      candidate.evidence_policy.ambiguity_exit_maximum_entropy = parameter.as_double();
    } else if (name == "health_policy.ambiguity_entry_minimum_effective_component_count") {
      candidate.evidence_policy.ambiguity_entry_minimum_effective_component_count = parameter.as_double();
    } else if (name == "health_policy.ambiguity_exit_maximum_effective_component_count") {
      candidate.evidence_policy.ambiguity_exit_maximum_effective_component_count = parameter.as_double();
    } else if (name == "health_policy.ambiguity_entry_maximum_dominant_weight") {
      candidate.evidence_policy.ambiguity_entry_maximum_dominant_weight = parameter.as_double();
    } else if (name == "health_policy.ambiguity_exit_minimum_dominant_weight") {
      candidate.evidence_policy.ambiguity_exit_minimum_dominant_weight = parameter.as_double();
    } else if (name == "health_policy.ambiguity_component_budget_fraction") {
      candidate.evidence_policy.ambiguity_component_budget_fraction = parameter.as_double();
    } else if (name == "health_policy.maximum_component_count") {
      candidate.evidence_policy.maximum_component_count = parameter_count(parameter);
    }
  }

  validate(candidate);
  return candidate;
}

bool is_particle_analysis_parameter(const std::string & name) noexcept
{
  return name.rfind("particle_clustering.", 0U) == 0U ||
         name.rfind("health_policy.", 0U) == 0U;
}

}  // namespace hybrid_localization_ros
