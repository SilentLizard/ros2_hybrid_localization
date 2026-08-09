#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hybrid_localization_msgs/msg/gaussian_mixture.hpp"
#include "hybrid_localization_msgs/msg/localization_health.hpp"
#include "hybrid_localization_msgs/msg/particle_analysis.hpp"
#include "hybrid_localization_msgs/msg/transition_evidence.hpp"
#include "hybrid_localization_ros/amcl_particle_cloud_adapter.hpp"
#include "hybrid_localization_ros/particle_analysis_parameters.hpp"
#include "hybrid_localization_ros/particle_analysis_processor.hpp"

namespace hybrid_localization_ros
{

class ParticleAnalysisNode final : public rclcpp::Node
{
public:
  ParticleAnalysisNode()
  : Node("particle_analysis_observer")
   {
    const auto initial_config = declare_particle_analysis_parameters(*this);
    processor_ = std::make_unique<ParticleAnalysisProcessor>(initial_config);

    parameter_validation_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return validate_parameter_update(parameters);
      });

    parameter_post_set_callback_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        apply_parameter_update(parameters);
      });
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
      "Particle-analysis observation node started with runtime clustering/health parameters; "
      "AMCL remains authoritative");
  }

private:
  rcl_interfaces::msg::SetParametersResult validate_parameter_update(
    const std::vector<rclcpp::Parameter> & parameters) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool relevant = false;
    for (const auto & parameter : parameters) {
      relevant = relevant || is_particle_analysis_parameter(parameter.get_name());
    }
    if (!relevant) {
      return result;
    }

    try {
      (void)apply_particle_analysis_parameter_updates(processor_->config(), parameters);
    } catch (const std::exception & error) {
      result.successful = false;
      result.reason = error.what();
    }
    return result;
  }

  void apply_parameter_update(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool relevant = false;
    for (const auto & parameter : parameters) {
      relevant = relevant || is_particle_analysis_parameter(parameter.get_name());
    }
    if (!relevant) {
      return;
    }

    // The on-set callback already validated this complete transaction. The
    // post-set callback is the correct place to perform the runtime side effect
    // after ROS has accepted and stored the new parameter values.
    const auto config = apply_particle_analysis_parameter_updates(
      processor_->config(), parameters);
    processor_->set_config(config);

    RCLCPP_INFO(
      get_logger(),
      "Applied particle-analysis runtime parameter update (%zu parameter(s))",
      parameters.size());
  }

  void handle_cloud(AdaptedParticleCloud cloud)
  {
    try {
      auto products = processor_->process(cloud);

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

  std::unique_ptr<ParticleAnalysisProcessor> processor_;
  std::unique_ptr<AmclParticleCloudAdapter> adapter_;
  
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_validation_callback_;
  rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr parameter_post_set_callback_;

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
