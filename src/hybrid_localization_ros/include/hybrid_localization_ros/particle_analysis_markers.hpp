#pragma once

#include <cstddef>

#include "hybrid_localization_msgs/msg/particle_analysis.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace hybrid_localization_ros
{

/// Configuration for observation-mode RViz marker generation.
struct ParticleAnalysisMarkerConfig
{
  /// Number of standard deviations represented by the XY covariance ellipse.
  double covariance_sigma{2.0};
  double ellipse_z{0.05};
  double mean_marker_diameter{0.12};
  double heading_length{0.35};
  double text_height{0.13};
  double status_text_height{0.16};
  double label_offset_radius{0.24};
  double health_offset_x{1.15};
  double health_offset_y{0.65};
  double authority_offset_x{1.15};
  double authority_offset_y{0.10};
  std::size_t ellipse_segments{64U};
};

/// Build deterministic observation-mode markers from one ParticleAnalysis.
///
/// previous_component_count is used to emit DELETE markers for component slots
/// that existed in the previous update but no longer exist in this update.
/// This avoids DELETEALL and the associated RViz flicker.
[[nodiscard]] visualization_msgs::msg::MarkerArray build_particle_analysis_markers(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  std::size_t previous_component_count = 0U,
  const ParticleAnalysisMarkerConfig & config = {});

}  // namespace hybrid_localization_ros
