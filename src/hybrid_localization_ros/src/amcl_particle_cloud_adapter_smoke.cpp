#include <cstddef>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "hybrid_localization_ros/amcl_particle_cloud_adapter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("amcl_particle_cloud_adapter_smoke");
  std::size_t received_clouds = 0;

  hybrid_localization_ros::AmclParticleCloudAdapter adapter(
    *node,
    [&node, &received_clouds](hybrid_localization_ros::AdaptedParticleCloud cloud) {
      ++received_clouds;
      if (received_clouds == 1 || received_clouds % 20 == 0) {
        RCLCPP_INFO(
          node->get_logger(),
          "Accepted AMCL cloud #%zu: frame=%s particles=%zu source_weight_sum=%.17g",
          received_clouds,
          cloud.header.frame_id.c_str(),
          cloud.particles.size(),
          cloud.source_weight_sum);
      }
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
