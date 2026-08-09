#pragma once

#include <span>

#include "rclcpp/rclcpp.hpp"

#include "hybrid_localization_ros/particle_analysis_processor.hpp"

namespace hybrid_localization_ros
{

/// Declare the ROS parameters currently consumed by particle-analysis
/// observation mode and return the validated initial processor configuration.
[[nodiscard]] ParticleAnalysisProcessorConfig declare_particle_analysis_parameters(
  rclcpp::Node & node,
  const ParticleAnalysisProcessorConfig & defaults = {});

/// Apply a prospective ROS parameter transaction to a processor configuration.
///
/// Parameters outside the particle-analysis namespaces are ignored. The
/// resulting complete configuration is validated through the core validators
/// before it is returned.
[[nodiscard]] ParticleAnalysisProcessorConfig apply_particle_analysis_parameter_updates(
  const ParticleAnalysisProcessorConfig & current,
  std::span<const rclcpp::Parameter> parameters);

/// Return true when the parameter belongs to the currently exposed #11 groups.
[[nodiscard]] bool is_particle_analysis_parameter(const std::string & name) noexcept;

}  // namespace hybrid_localization_ros
