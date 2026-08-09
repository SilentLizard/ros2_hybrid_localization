#include <cstddef>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "hybrid_localization_msgs/msg/particle_analysis.hpp"
#include "hybrid_localization_ros/particle_analysis_markers.hpp"

namespace hybrid_localization_ros
{

class ParticleAnalysisVisualizationNode final : public rclcpp::Node
{
public:
  ParticleAnalysisVisualizationNode()
  : Node("particle_analysis_visualization")
  {
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "hybrid_localization/rviz_markers", output_qos);

    analysis_subscription_ =
      create_subscription<hybrid_localization_msgs::msg::ParticleAnalysis>(
      "hybrid_localization/particle_analysis",
      input_qos,
      [this](const hybrid_localization_msgs::msg::ParticleAnalysis::SharedPtr message) {
        handle_analysis(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Particle-analysis RViz visualization started; markers are observation-only and do not own TF authority");
  }

private:
  void handle_analysis(const hybrid_localization_msgs::msg::ParticleAnalysis & analysis)
  {
    try {
      auto markers = build_particle_analysis_markers(analysis, previous_component_count_);
      previous_component_count_ = analysis.mixture.components.size();
      marker_publisher_->publish(std::move(markers));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Could not build particle-analysis RViz markers: %s", error.what());
    }
  }

  std::size_t previous_component_count_{0U};
  rclcpp::Subscription<hybrid_localization_msgs::msg::ParticleAnalysis>::SharedPtr
    analysis_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}  // namespace hybrid_localization_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybrid_localization_ros::ParticleAnalysisVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
