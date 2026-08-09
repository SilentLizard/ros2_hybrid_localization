#include <cstddef>
#include <exception>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "hybrid_localization_msgs/msg/gaussian_mixture.hpp"
#include "hybrid_localization_msgs/msg/localization_health.hpp"
#include "hybrid_localization_msgs/msg/particle_analysis.hpp"
#include "hybrid_localization_msgs/msg/transition_evidence.hpp"
#include "hybrid_localization_ros/amcl_particle_cloud_adapter.hpp"
#include "hybrid_localization_ros/particle_analysis_processor.hpp"

namespace hybrid_localization_ros
{

class ParticleAnalysisNode final : public rclcpp::Node
{
public:
  ParticleAnalysisNode()
  : Node("particle_analysis_observer"),
    processor_()
  {
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    analysis_publisher_ = create_publisher<hybrid_localization_msgs::msg::ParticleAnalysis>(
      "hybrid_localization/particle_analysis",
      output_qos);
    mixture_publisher_ = create_publisher<hybrid_localization_msgs::msg::GaussianMixture>(
      "hybrid_localization/gaussian_mixture",
      output_qos);
    health_publisher_ = create_publisher<hybrid_localization_msgs::msg::LocalizationHealth>(
      "hybrid_localization/localization_health",
      output_qos);
    evidence_publisher_ = create_publisher<hybrid_localization_msgs::msg::TransitionEvidence>(
      "hybrid_localization/transition_evidence",
      output_qos);

    adapter_ = std::make_unique<AmclParticleCloudAdapter>(
      *this,
      [this](AdaptedParticleCloud cloud) {
        handle_cloud(std::move(cloud));
      });

    RCLCPP_INFO(
      get_logger(),
      "Particle-analysis observation node started; AMCL remains authoritative");
  }

private:
  void handle_cloud(AdaptedParticleCloud cloud)
  {
    try {
      auto products = processor_.process(cloud);

      mixture_publisher_->publish(products.mixture);
      health_publisher_->publish(products.health);
      evidence_publisher_->publish(products.evidence);
      analysis_publisher_->publish(products.analysis);

      const auto sequence = products.analysis.analysis_sequence;
      if (sequence == 1U || sequence % 20U == 0U) {
        RCLCPP_INFO(
          get_logger(),
          "Particle analysis #%llu: particles=%llu clusters=%llu represented=%.6f noise=%.6f",
          static_cast<unsigned long long>(sequence),
          static_cast<unsigned long long>(products.analysis.particle_count),
          static_cast<unsigned long long>(products.analysis.retained_cluster_count),
          products.analysis.health.represented_weight,
          products.analysis.noise_weight);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Particle analysis rejected an adapted cloud: %s",
        error.what());
    }
  }

  ParticleAnalysisProcessor processor_;
  std::unique_ptr<AmclParticleCloudAdapter> adapter_;

  rclcpp::Publisher<hybrid_localization_msgs::msg::ParticleAnalysis>::SharedPtr
    analysis_publisher_;
  rclcpp::Publisher<hybrid_localization_msgs::msg::GaussianMixture>::SharedPtr
    mixture_publisher_;
  rclcpp::Publisher<hybrid_localization_msgs::msg::LocalizationHealth>::SharedPtr
    health_publisher_;
  rclcpp::Publisher<hybrid_localization_msgs::msg::TransitionEvidence>::SharedPtr
    evidence_publisher_;
};

}  // namespace hybrid_localization_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybrid_localization_ros::ParticleAnalysisNode>());
  rclcpp::shutdown();
  return 0;
}
