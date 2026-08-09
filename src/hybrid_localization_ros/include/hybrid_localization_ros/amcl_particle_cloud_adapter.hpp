#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "nav2_msgs/msg/particle_cloud.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization_ros
{

/// Validation policy for the Nav2 AMCL particle-cloud boundary.
struct AmclParticleCloudAdapterConfig
{
  std::string expected_frame_id{"map"};
  double planar_position_tolerance{1e-9};
  double planar_orientation_tolerance{1e-9};
  double quaternion_norm_tolerance{1e-6};
};

/// ROS metadata plus the normalized ROS-independent particle representation.
struct AdaptedParticleCloud
{
  std_msgs::msg::Header header{};
  std::vector<hybrid_localization::WeightedParticle> particles{};
  double source_weight_sum{0.0};
};

/// Convert one Nav2 ParticleCloud into validated, normalized core particles.
///
/// The adapter preserves the source header, rejects malformed/non-planar data,
/// converts quaternion orientation to wrapped SE(2) yaw, and normalizes particle
/// weights through hybrid_localization_core before returning.
///
/// Throws std::invalid_argument for an invalid configuration or message.
[[nodiscard]] AdaptedParticleCloud adapt_amcl_particle_cloud(
  const nav2_msgs::msg::ParticleCloud & message,
  const AmclParticleCloudAdapterConfig & config = {});

/// QoS matching the runtime contract validated for Nav2 Jazzy AMCL (#41).
[[nodiscard]] rclcpp::QoS amcl_particle_cloud_qos();

/// Reusable subscription boundary between Nav2 AMCL and hybrid localization.
///
/// The class owns only the subscription. A consumer callback receives each
/// successfully adapted cloud, allowing the later particle-analysis node to use
/// the adapter without adding an intermediate ROS topic or serialization step.
class AmclParticleCloudAdapter
{
public:
  using Consumer = std::function<void(AdaptedParticleCloud)>;

  AmclParticleCloudAdapter(
    rclcpp::Node & node,
    Consumer consumer,
    AmclParticleCloudAdapterConfig config = {},
    std::string topic = "/particle_cloud");

  [[nodiscard]] std::size_t accepted_cloud_count() const noexcept;
  [[nodiscard]] std::size_t rejected_cloud_count() const noexcept;

private:
  void handle_message(
    const nav2_msgs::msg::ParticleCloud::SharedPtr message);

  rclcpp::Logger logger_;
  Consumer consumer_;
  AmclParticleCloudAdapterConfig config_;
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr subscription_;
  std::size_t accepted_cloud_count_{0};
  std::size_t rejected_cloud_count_{0};
};

}  // namespace hybrid_localization_ros
