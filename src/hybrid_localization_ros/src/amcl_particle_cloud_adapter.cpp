#include "hybrid_localization_ros/amcl_particle_cloud_adapter.hpp"

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hybrid_localization_core/geometry.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"

namespace hybrid_localization_ros
{
namespace
{

void validate_config(const AmclParticleCloudAdapterConfig & config)
{
  if (config.expected_frame_id.empty()) {
    throw std::invalid_argument("Expected particle-cloud frame must not be empty");
  }

  const auto valid_tolerance = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };

  if (!valid_tolerance(config.planar_position_tolerance)) {
    throw std::invalid_argument(
            "Planar position tolerance must be finite and non-negative");
  }
  if (!valid_tolerance(config.planar_orientation_tolerance)) {
    throw std::invalid_argument(
            "Planar orientation tolerance must be finite and non-negative");
  }
  if (!valid_tolerance(config.quaternion_norm_tolerance)) {
    throw std::invalid_argument(
            "Quaternion norm tolerance must be finite and non-negative");
  }
}

[[nodiscard]] double quaternion_to_yaw(
  const geometry_msgs::msg::Quaternion & orientation,
  const AmclParticleCloudAdapterConfig & config,
  const std::size_t particle_index)
{
  const double x = orientation.x;
  const double y = orientation.y;
  const double z = orientation.z;
  const double w = orientation.w;

  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z) || !std::isfinite(w))
  {
    throw std::invalid_argument(
            "Particle quaternion must be finite at index " +
            std::to_string(particle_index));
  }

  if (std::abs(x) > config.planar_orientation_tolerance ||
      std::abs(y) > config.planar_orientation_tolerance)
  {
    throw std::invalid_argument(
            "Particle quaternion must represent planar yaw at index " +
            std::to_string(particle_index));
  }

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) ||
      std::abs(norm - 1.0) > config.quaternion_norm_tolerance)
  {
    throw std::invalid_argument(
            "Particle quaternion must be normalized at index " +
            std::to_string(particle_index));
  }

  // General quaternion-to-yaw expression. The explicit planar checks above
  // ensure roll/pitch leakage cannot silently enter the SE(2) core.
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  return hybrid_localization::normalize_angle(std::atan2(sin_yaw, cos_yaw));
}

}  // namespace

AdaptedParticleCloud adapt_amcl_particle_cloud(
  const nav2_msgs::msg::ParticleCloud & message,
  const AmclParticleCloudAdapterConfig & config)
{
  validate_config(config);

  if (message.header.frame_id != config.expected_frame_id) {
    throw std::invalid_argument(
            "Unexpected particle-cloud frame: expected '" +
            config.expected_frame_id + "' but received '" +
            message.header.frame_id + "'");
  }

  if (message.particles.empty()) {
    throw std::invalid_argument("Particle cloud must not be empty");
  }

  std::vector<hybrid_localization::WeightedParticle> particles;
  particles.reserve(message.particles.size());

  double source_weight_sum = 0.0;

  for (std::size_t index = 0; index < message.particles.size(); ++index) {
    const auto & source = message.particles[index];
    const auto & position = source.pose.position;

    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      throw std::invalid_argument(
              "Particle position must be finite at index " +
              std::to_string(index));
    }

    if (std::abs(position.z) > config.planar_position_tolerance) {
      throw std::invalid_argument(
              "Particle position must be planar at index " +
              std::to_string(index));
    }

    if (!std::isfinite(source.weight) || source.weight < 0.0) {
      throw std::invalid_argument(
              "Particle weight must be finite and non-negative at index " +
              std::to_string(index));
    }

    source_weight_sum += source.weight;
    if (!std::isfinite(source_weight_sum)) {
      throw std::invalid_argument("Particle weight sum must remain finite");
    }

    particles.push_back({
      .pose = {
        .x = position.x,
        .y = position.y,
        .yaw = quaternion_to_yaw(source.pose.orientation, config, index),
      },
      .weight = source.weight,
    });
  }

  // normalize_weights() also rejects zero total mass and serves as the final
  // core-side validation boundary for finite poses and non-negative weights.
  auto normalized = hybrid_localization::normalize_weights(particles);

  return {
    .header = message.header,
    .particles = std::move(normalized),
    .source_weight_sum = source_weight_sum,
  };
}

rclcpp::QoS amcl_particle_cloud_qos()
{
  return rclcpp::SensorDataQoS();
}

AmclParticleCloudAdapter::AmclParticleCloudAdapter(
  rclcpp::Node & node,
  Consumer consumer,
  AmclParticleCloudAdapterConfig config,
  std::string topic)
: logger_(node.get_logger()),
  consumer_(std::move(consumer)),
  config_(std::move(config))
{
  validate_config(config_);

  if (!consumer_) {
    throw std::invalid_argument("Particle-cloud consumer callback must be set");
  }
  if (topic.empty()) {
    throw std::invalid_argument("Particle-cloud topic must not be empty");
  }

  subscription_ = node.create_subscription<nav2_msgs::msg::ParticleCloud>(
    std::move(topic),
    amcl_particle_cloud_qos(),
    [this](const nav2_msgs::msg::ParticleCloud::SharedPtr message) {
      handle_message(message);
    });
}

std::size_t AmclParticleCloudAdapter::accepted_cloud_count() const noexcept
{
  return accepted_cloud_count_;
}

std::size_t AmclParticleCloudAdapter::rejected_cloud_count() const noexcept
{
  return rejected_cloud_count_;
}

void AmclParticleCloudAdapter::handle_message(
  const nav2_msgs::msg::ParticleCloud::SharedPtr message)
{
  AdaptedParticleCloud adapted;
  try {
    adapted = adapt_amcl_particle_cloud(*message, config_);
  } catch (const std::invalid_argument & error) {
    ++rejected_cloud_count_;
    RCLCPP_WARN(
      logger_,
      "Rejected AMCL particle cloud: %s",
      error.what());
    return;
  }

  ++accepted_cloud_count_;
  consumer_(std::move(adapted));
}

}  // namespace hybrid_localization_ros
