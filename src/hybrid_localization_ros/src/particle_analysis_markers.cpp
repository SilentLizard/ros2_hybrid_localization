#include "hybrid_localization_ros/particle_analysis_markers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace hybrid_localization_ros
{
namespace
{

using visualization_msgs::msg::Marker;

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kCovarianceTolerance = 1e-12;

[[nodiscard]] std_msgs::msg::ColorRGBA color(
  const float red,
  const float green,
  const float blue,
  const float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA result;
  result.r = red;
  result.g = green;
  result.b = blue;
  result.a = alpha;
  return result;
}

[[nodiscard]] std_msgs::msg::ColorRGBA component_color(const std::size_t index)
{
  constexpr std::array<std::array<float, 3>, 6> palette{{
    {{0.15F, 0.75F, 1.00F}},
    {{1.00F, 0.55F, 0.15F}},
    {{0.45F, 0.90F, 0.35F}},
    {{0.90F, 0.35F, 0.80F}},
    {{1.00F, 0.85F, 0.20F}},
    {{0.55F, 0.55F, 1.00F}},
  }};
  const auto & rgb = palette[index % palette.size()];
  return color(rgb[0], rgb[1], rgb[2]);
}

void require_finite(const double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void validate_config(const ParticleAnalysisMarkerConfig & config)
{
  require_finite(config.covariance_sigma, "covariance_sigma");
  require_finite(config.ellipse_z, "ellipse_z");
  require_finite(config.mean_marker_diameter, "mean_marker_diameter");
  require_finite(config.heading_length, "heading_length");
  require_finite(config.text_height, "text_height");
  require_finite(config.status_text_height, "status_text_height");
  require_finite(config.label_offset_radius, "label_offset_radius");
  require_finite(config.health_offset_x, "health_offset_x");
  require_finite(config.health_offset_y, "health_offset_y");
  require_finite(config.authority_offset_x, "authority_offset_x");
  require_finite(config.authority_offset_y, "authority_offset_y");

  if (config.covariance_sigma <= 0.0 ||
      config.mean_marker_diameter <= 0.0 ||
      config.heading_length <= 0.0 ||
      config.text_height <= 0.0 ||
      config.status_text_height <= 0.0 ||
      config.label_offset_radius < 0.0)
  {
    throw std::invalid_argument("RViz marker scales must be positive");
  }
  if (config.ellipse_segments < 12U) {
    throw std::invalid_argument("ellipse_segments must be at least 12");
  }
}

[[nodiscard]] Marker base_marker(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  std::string ns,
  const std::int32_t id,
  const std::int32_t type)
{
  Marker marker;
  marker.header = analysis.header;
  marker.ns = std::move(ns);
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

[[nodiscard]] std::int32_t checked_marker_id(const std::size_t index)
{
  if (index > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::overflow_error("GMM component index exceeds RViz marker ID range");
  }
  return static_cast<std::int32_t>(index);
}

[[nodiscard]] std::array<double, 3> xy_covariance_axes(
  const std::array<double, 9> & covariance)
{
  const double a = covariance[0];
  const double b = 0.5 * (covariance[1] + covariance[3]);
  const double d = covariance[4];

  require_finite(a, "covariance xx");
  require_finite(b, "covariance xy");
  require_finite(d, "covariance yy");

  const double discriminant = std::hypot(a - d, 2.0 * b);
  double lambda_major = 0.5 * (a + d + discriminant);
  double lambda_minor = 0.5 * (a + d - discriminant);

  if (lambda_major < -kCovarianceTolerance || lambda_minor < -kCovarianceTolerance) {
    throw std::invalid_argument("XY covariance must be positive semidefinite");
  }
  lambda_major = std::max(0.0, lambda_major);
  lambda_minor = std::max(0.0, lambda_minor);

  const double angle = 0.5 * std::atan2(2.0 * b, a - d);
  return {std::sqrt(lambda_major), std::sqrt(lambda_minor), angle};
}

[[nodiscard]] Marker covariance_marker(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const hybrid_localization_msgs::msg::GaussianComponent & component,
  const std::size_t index,
  const ParticleAnalysisMarkerConfig & config)
{
  Marker marker = base_marker(
    analysis, "gmm_covariance", checked_marker_id(index), Marker::LINE_STRIP);
  marker.scale.x = 0.035;
  marker.color = component_color(index);

  const auto axes = xy_covariance_axes(component.covariance);
  const double major = config.covariance_sigma * axes[0];
  const double minor = config.covariance_sigma * axes[1];
  const double c = std::cos(axes[2]);
  const double s = std::sin(axes[2]);

  marker.points.reserve(config.ellipse_segments + 1U);
  for (std::size_t segment = 0U; segment <= config.ellipse_segments; ++segment) {
    const double phase = 2.0 * kPi * static_cast<double>(segment) /
      static_cast<double>(config.ellipse_segments);
    const double local_x = major * std::cos(phase);
    const double local_y = minor * std::sin(phase);

    geometry_msgs::msg::Point point;
    point.x = component.pose.position.x + c * local_x - s * local_y;
    point.y = component.pose.position.y + s * local_x + c * local_y;
    point.z = config.ellipse_z;
    marker.points.push_back(point);
  }
  return marker;
}

[[nodiscard]] Marker mean_marker(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const hybrid_localization_msgs::msg::GaussianComponent & component,
  const std::size_t index,
  const ParticleAnalysisMarkerConfig & config)
{
  Marker marker = base_marker(
    analysis, "gmm_mean", checked_marker_id(index), Marker::SPHERE);
  marker.pose = component.pose;
  marker.pose.position.z = config.ellipse_z;
  marker.scale.x = config.mean_marker_diameter;
  marker.scale.y = config.mean_marker_diameter;
  marker.scale.z = config.mean_marker_diameter;
  marker.color = component_color(index);
  return marker;
}

[[nodiscard]] Marker heading_marker(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const hybrid_localization_msgs::msg::GaussianComponent & component,
  const std::size_t index,
  const ParticleAnalysisMarkerConfig & config)
{
  Marker marker = base_marker(
    analysis, "gmm_heading", checked_marker_id(index), Marker::ARROW);
  marker.pose = component.pose;
  marker.pose.position.z = config.ellipse_z;
  marker.scale.x = config.heading_length;
  marker.scale.y = 0.055;
  marker.scale.z = 0.08;
  marker.color = component_color(index);
  return marker;
}

[[nodiscard]] Marker label_marker(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const hybrid_localization_msgs::msg::GaussianComponent & component,
  const std::size_t index,
  const ParticleAnalysisMarkerConfig & config)
{
  Marker marker = base_marker(
    analysis, "gmm_label", checked_marker_id(index), Marker::TEXT_VIEW_FACING);
  marker.pose.position = component.pose.position;

  // TopDownOrtho projects different Z values onto nearly the same screen
  // location, so separate labels in XY instead of stacking them vertically.
  constexpr double golden_angle = 2.39996322972865332;
  const double label_angle = golden_angle * static_cast<double>(index);
  marker.pose.position.x += config.label_offset_radius * std::cos(label_angle);
  marker.pose.position.y += config.label_offset_radius * std::sin(label_angle);
  marker.pose.position.z = config.ellipse_z + 0.16;

  
  marker.scale.z = config.text_height;
  marker.color = component_color(index);

  std::ostringstream stream;
  stream.precision(3);
  stream << "H" << component.provenance.id
         << "  w=" << std::fixed << component.weight
         << "  n=" << component.sample_count;
  marker.text = stream.str();
  return marker;
}

[[nodiscard]] std::string health_text(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis)
{
  const auto & health = analysis.health;
  const auto & evidence = analysis.evidence;

  std::ostringstream stream;
  stream.precision(3);
  stream << "GMM health: ";
  if (evidence.gmm_health_bad) {
    stream << "BAD";
  } else if (evidence.gmm_health_good) {
    stream << "GOOD";
  } else {
    stream << "NEUTRAL";
  }
  if (evidence.tracking_ambiguous) {
    stream << " / AMBIGUOUS";
  }
  stream << "\ncomponents=" << health.component_count
         << " represented=" << std::fixed << health.represented_weight
         << " discarded=" << health.discarded_weight
         << " entropy=" << health.normalized_mixture_entropy;
  stream << "\nparticles=" << analysis.particle_count
         << " ESS=" << analysis.effective_sample_size
         << " clusters=" << analysis.retained_cluster_count
         << " noise=" << analysis.noise_weight;
  return stream.str();
}

[[nodiscard]] Marker health_marker(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const ParticleAnalysisMarkerConfig & config)
{
  Marker marker = base_marker(analysis, "hybrid_status", 0, Marker::TEXT_VIEW_FACING);
  
  marker.pose.position.x = (analysis.mixture.components.empty() ? 0.0 :
    analysis.mixture.components.front().pose.position.x) + config.health_offset_x;
  marker.pose.position.y = (analysis.mixture.components.empty() ? 0.0 :
    analysis.mixture.components.front().pose.position.y) + config.health_offset_y;
  marker.pose.position.z = config.ellipse_z + 0.12;
  
  marker.scale.z = config.status_text_height;
  marker.text = health_text(analysis);

  if (analysis.evidence.gmm_health_bad) {
    marker.color = color(1.0F, 0.25F, 0.2F);
  } else if (analysis.evidence.gmm_health_good) {
    marker.color = color(0.25F, 1.0F, 0.35F);
  } else {
    marker.color = color(1.0F, 0.85F, 0.25F);
  }
  return marker;
}

[[nodiscard]] Marker authority_marker(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const ParticleAnalysisMarkerConfig & config)
{
  Marker marker = base_marker(analysis, "hybrid_status", 1, Marker::TEXT_VIEW_FACING);
  marker.pose.position.x = (analysis.mixture.components.empty() ? 0.0 :
    analysis.mixture.components.front().pose.position.x) + config.authority_offset_x;
  marker.pose.position.y = (analysis.mixture.components.empty() ? 0.0 :
    analysis.mixture.components.front().pose.position.y) + config.authority_offset_y;
  marker.pose.position.z = config.ellipse_z + 0.12;
  marker.scale.z = config.status_text_height;
  marker.color = color(0.35F, 0.85F, 1.0F);
  marker.text = "Authority: AMCL / particles\nHybrid output: observation only";
  return marker;
}

void append_delete_markers(
  visualization_msgs::msg::MarkerArray & markers,
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const std::size_t first_deleted,
  const std::size_t previous_component_count)
{
  constexpr std::array<const char *, 4> namespaces{
    "gmm_covariance", "gmm_mean", "gmm_heading", "gmm_label"};

  for (std::size_t index = first_deleted; index < previous_component_count; ++index) {
    for (const char * ns : namespaces) {
      Marker marker = base_marker(analysis, ns, checked_marker_id(index), Marker::SPHERE);
      marker.action = Marker::DELETE;
      markers.markers.push_back(std::move(marker));
    }
  }
}

}  // namespace

visualization_msgs::msg::MarkerArray build_particle_analysis_markers(
  const hybrid_localization_msgs::msg::ParticleAnalysis & analysis,
  const std::size_t previous_component_count,
  const ParticleAnalysisMarkerConfig & config)
{
  validate_config(config);

  visualization_msgs::msg::MarkerArray markers;
  const auto & components = analysis.mixture.components;
  markers.markers.reserve(components.size() * 4U + 2U +
    (previous_component_count > components.size() ?
      (previous_component_count - components.size()) * 4U : 0U));

  for (std::size_t index = 0U; index < components.size(); ++index) {
    const auto & component = components[index];
    markers.markers.push_back(covariance_marker(analysis, component, index, config));
    markers.markers.push_back(mean_marker(analysis, component, index, config));
    markers.markers.push_back(heading_marker(analysis, component, index, config));
    markers.markers.push_back(label_marker(analysis, component, index, config));
  }

  if (previous_component_count > components.size()) {
    append_delete_markers(markers, analysis, components.size(), previous_component_count);
  }

  markers.markers.push_back(health_marker(analysis, config));
  markers.markers.push_back(authority_marker(analysis, config));
  return markers;
}

}  // namespace hybrid_localization_ros
