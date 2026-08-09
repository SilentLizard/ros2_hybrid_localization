#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "visualization_msgs/msg/marker.hpp"

#include "hybrid_localization_msgs/msg/gaussian_component.hpp"
#include "hybrid_localization_msgs/msg/particle_analysis.hpp"
#include "hybrid_localization_ros/particle_analysis_markers.hpp"

namespace hlr = hybrid_localization_ros;
namespace hlm = hybrid_localization_msgs::msg;
namespace vm = visualization_msgs::msg;

namespace
{

hlm::GaussianComponent component(
  const double x,
  const double y,
  const double yaw,
  const double xx,
  const double xy,
  const double yy,
  const double weight,
  const std::uint64_t id)
{
  hlm::GaussianComponent result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(0.5 * yaw);
  result.pose.orientation.w = std::cos(0.5 * yaw);
  result.covariance = {xx, xy, 0.0, xy, yy, 0.0, 0.0, 0.0, 0.04};
  result.weight = weight;
  result.sample_count = 50U;
  result.provenance.id = id;
  return result;
}

hlm::ParticleAnalysis analysis_with_components(const std::size_t count)
{
  hlm::ParticleAnalysis result;
  result.header.frame_id = "map";
  result.header.stamp.sec = 123;
  result.header.stamp.nanosec = 456U;
  result.analysis_sequence = 9U;
  result.particle_count = 100U;
  result.effective_sample_size = 80.0;
  result.retained_cluster_count = count;
  result.retained_cluster_weight = 1.0;
  result.dominant_cluster_weight = count == 0U ? 0.0 : 1.0;
  result.mixture.header = result.header;
  result.mixture.update_sequence = result.analysis_sequence;
  result.health.header = result.header;
  result.health.update_sequence = result.analysis_sequence;
  result.health.component_count = count;
  result.health.represented_weight = count == 0U ? 0.0 : 1.0;
  result.health.dominant_component_weight = count == 0U ? 0.0 : 1.0;
  result.evidence.gmm_health_good = count > 0U;
  result.evidence.gmm_available = count > 0U;

  for (std::size_t index = 0U; index < count; ++index) {
    result.mixture.components.push_back(component(
      static_cast<double>(index),
      0.5 * static_cast<double>(index),
      0.2 * static_cast<double>(index),
      0.25,
      0.05,
      0.09,
      1.0 / static_cast<double>(count),
      100U + index));
  }
  return result;
}

const vm::Marker * find_marker(
  const vm::MarkerArray & array,
  const std::string & ns,
  const std::int32_t id,
  const std::int32_t action = vm::Marker::ADD)
{
  const auto it = std::find_if(
    array.markers.begin(), array.markers.end(),
    [&](const vm::Marker & marker) {
      return marker.ns == ns && marker.id == id && marker.action == action;
    });
  return it == array.markers.end() ? nullptr : &*it;
}

}  // namespace

TEST(ParticleAnalysisMarkers, BuildsDeterministicMarkersAndPreservesHeader)
{
  const auto analysis = analysis_with_components(2U);
  const auto markers = hlr::build_particle_analysis_markers(analysis);

  EXPECT_EQ(markers.markers.size(), 10U);

  const auto * ellipse = find_marker(markers, "gmm_covariance", 0);
  ASSERT_NE(ellipse, nullptr);
  EXPECT_EQ(ellipse->header.frame_id, "map");
  EXPECT_EQ(ellipse->header.stamp.sec, 123);
  EXPECT_EQ(ellipse->header.stamp.nanosec, 456U);
  EXPECT_EQ(ellipse->type, vm::Marker::LINE_STRIP);
  EXPECT_EQ(ellipse->points.size(), 65U);

  const auto * label = find_marker(markers, "gmm_label", 1);
  ASSERT_NE(label, nullptr);
  EXPECT_NE(label->text.find("H101"), std::string::npos);

  EXPECT_NE(find_marker(markers, "hybrid_status", 0), nullptr);
  EXPECT_NE(find_marker(markers, "hybrid_status", 1), nullptr);
}

TEST(ParticleAnalysisMarkers, CovarianceEllipseUsesRequestedSigma)
{
  auto analysis = analysis_with_components(1U);
  analysis.mixture.components[0].pose.position.x = 1.0;
  analysis.mixture.components[0].pose.position.y = 2.0;
  analysis.mixture.components[0].covariance = {
    0.25, 0.0, 0.0,
    0.0, 0.09, 0.0,
    0.0, 0.0, 0.04};

  const auto markers = hlr::build_particle_analysis_markers(analysis);
  const auto * ellipse = find_marker(markers, "gmm_covariance", 0);
  ASSERT_NE(ellipse, nullptr);
  ASSERT_FALSE(ellipse->points.empty());

  // First point lies on the +major axis: 2 sigma * sqrt(0.25) = 1.0 m.
  EXPECT_NEAR(ellipse->points.front().x, 2.0, 1e-12);
  EXPECT_NEAR(ellipse->points.front().y, 2.0, 1e-12);
}

TEST(ParticleAnalysisMarkers, DeletesOnlyStaleComponentSlots)
{
  const auto analysis = analysis_with_components(1U);
  const auto markers = hlr::build_particle_analysis_markers(analysis, 3U);

  EXPECT_EQ(markers.markers.size(), 14U);
  for (std::int32_t id = 1; id <= 2; ++id) {
    EXPECT_NE(find_marker(markers, "gmm_covariance", id, vm::Marker::DELETE), nullptr);
    EXPECT_NE(find_marker(markers, "gmm_mean", id, vm::Marker::DELETE), nullptr);
    EXPECT_NE(find_marker(markers, "gmm_heading", id, vm::Marker::DELETE), nullptr);
    EXPECT_NE(find_marker(markers, "gmm_label", id, vm::Marker::DELETE), nullptr);
  }
  EXPECT_EQ(find_marker(markers, "gmm_mean", 0, vm::Marker::DELETE), nullptr);
}

TEST(ParticleAnalysisMarkers, AuthorityMarkerStatesObservationModeExplicitly)
{
  const auto analysis = analysis_with_components(1U);
  const auto markers = hlr::build_particle_analysis_markers(analysis);
  const auto * marker = find_marker(markers, "hybrid_status", 1);
  ASSERT_NE(marker, nullptr);
  EXPECT_NE(marker->text.find("AMCL / particles"), std::string::npos);
  EXPECT_NE(marker->text.find("observation only"), std::string::npos);
}

TEST(ParticleAnalysisMarkers, SeparatesLabelsAndStatusTextInTopDownView)
{
  const auto analysis = analysis_with_components(2U);
  const auto markers = hlr::build_particle_analysis_markers(analysis);

  const auto * label0 = find_marker(markers, "gmm_label", 0);
  const auto * label1 = find_marker(markers, "gmm_label", 1);
  const auto * health = find_marker(markers, "hybrid_status", 0);
  const auto * authority = find_marker(markers, "hybrid_status", 1);
  ASSERT_NE(label0, nullptr);
  ASSERT_NE(label1, nullptr);
  ASSERT_NE(health, nullptr);
  ASSERT_NE(authority, nullptr);

  EXPECT_TRUE(label0->pose.position.x != label1->pose.position.x ||
    label0->pose.position.y != label1->pose.position.y);
  EXPECT_TRUE(health->pose.position.x != authority->pose.position.x ||
    health->pose.position.y != authority->pose.position.y);
}

TEST(ParticleAnalysisMarkers, RejectsIndefiniteXyCovariance)
{
  auto analysis = analysis_with_components(1U);
  analysis.mixture.components[0].covariance = {
    0.01, 0.1, 0.0,
    0.1, 0.01, 0.0,
    0.0, 0.0, 0.01};

  EXPECT_THROW(
    (void)hlr::build_particle_analysis_markers(analysis),
    std::invalid_argument);
}

TEST(ParticleAnalysisMarkers, RejectsInvalidMarkerConfiguration)
{
  const auto analysis = analysis_with_components(1U);
  hlr::ParticleAnalysisMarkerConfig config;
  config.ellipse_segments = 4U;

  EXPECT_THROW(
    (void)hlr::build_particle_analysis_markers(analysis, 0U, config),
    std::invalid_argument);
}
