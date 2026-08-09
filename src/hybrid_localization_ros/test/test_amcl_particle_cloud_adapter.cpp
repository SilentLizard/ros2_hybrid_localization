#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/msg/particle.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"

#include "hybrid_localization_ros/amcl_particle_cloud_adapter.hpp"

namespace hlr = hybrid_localization_ros;

namespace
{

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(0.5 * yaw);
  result.w = std::cos(0.5 * yaw);
  return result;
}

nav2_msgs::msg::Particle make_particle(
  const double x,
  const double y,
  const double yaw,
  const double weight)
{
  nav2_msgs::msg::Particle particle;
  particle.pose.position.x = x;
  particle.pose.position.y = y;
  particle.pose.orientation = quaternion_from_yaw(yaw);
  particle.weight = weight;
  return particle;
}

nav2_msgs::msg::ParticleCloud make_cloud()
{
  nav2_msgs::msg::ParticleCloud cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = 123;
  cloud.header.stamp.nanosec = 456U;
  cloud.particles.push_back(make_particle(1.0, 2.0, 0.25, 2.0));
  cloud.particles.push_back(make_particle(-3.0, 4.0, -0.5, 1.0));
  return cloud;
}

}  // namespace

TEST(AmclParticleCloudAdapter, ConvertsAndNormalizesParticles)
{
  const auto adapted = hlr::adapt_amcl_particle_cloud(make_cloud());

  ASSERT_EQ(adapted.particles.size(), 2U);
  EXPECT_EQ(adapted.header.frame_id, "map");
  EXPECT_EQ(adapted.header.stamp.sec, 123);
  EXPECT_EQ(adapted.header.stamp.nanosec, 456U);
  EXPECT_DOUBLE_EQ(adapted.source_weight_sum, 3.0);

  EXPECT_DOUBLE_EQ(adapted.particles[0].pose.x, 1.0);
  EXPECT_DOUBLE_EQ(adapted.particles[0].pose.y, 2.0);
  EXPECT_NEAR(adapted.particles[0].pose.yaw, 0.25, 1e-12);
  EXPECT_NEAR(adapted.particles[0].weight, 2.0 / 3.0, 1e-12);

  EXPECT_DOUBLE_EQ(adapted.particles[1].pose.x, -3.0);
  EXPECT_DOUBLE_EQ(adapted.particles[1].pose.y, 4.0);
  EXPECT_NEAR(adapted.particles[1].pose.yaw, -0.5, 1e-12);
  EXPECT_NEAR(adapted.particles[1].weight, 1.0 / 3.0, 1e-12);
}

TEST(AmclParticleCloudAdapter, HandlesQuaternionSignAndYawWrap)
{
  auto cloud = make_cloud();
  cloud.particles.clear();

  constexpr double yaw = 3.13;
  auto particle = make_particle(0.0, 0.0, yaw, 1.0);
  particle.pose.orientation.z *= -1.0;
  particle.pose.orientation.w *= -1.0;
  cloud.particles.push_back(particle);

  const auto adapted = hlr::adapt_amcl_particle_cloud(cloud);
  ASSERT_EQ(adapted.particles.size(), 1U);
  EXPECT_NEAR(adapted.particles.front().pose.yaw, yaw, 1e-12);
}

TEST(AmclParticleCloudAdapter, AcceptsVariableParticleCounts)
{
  auto cloud = make_cloud();
  cloud.particles.clear();

  for (std::size_t index = 0; index < 2000U; ++index) {
    cloud.particles.push_back(
      make_particle(static_cast<double>(index), 0.0, 0.0, 0.0005));
  }

  const auto adapted = hlr::adapt_amcl_particle_cloud(cloud);
  EXPECT_EQ(adapted.particles.size(), 2000U);
  EXPECT_NEAR(adapted.source_weight_sum, 1.0, 1e-12);
}

TEST(AmclParticleCloudAdapter, RejectsUnexpectedFrame)
{
  auto cloud = make_cloud();
  cloud.header.frame_id = "odom";

  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);
}

TEST(AmclParticleCloudAdapter, RejectsEmptyCloudAndInvalidWeights)
{
  auto cloud = make_cloud();
  cloud.particles.clear();
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);

  cloud = make_cloud();
  cloud.particles.front().weight = -0.1;
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);

  cloud = make_cloud();
  cloud.particles.front().weight = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);

  cloud = make_cloud();
  for (auto & particle : cloud.particles) {
    particle.weight = 0.0;
  }
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);
}

TEST(AmclParticleCloudAdapter, RejectsNonPlanarAndMalformedPose)
{
  auto cloud = make_cloud();
  cloud.particles.front().pose.position.z = 0.01;
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);

  cloud = make_cloud();
  cloud.particles.front().pose.orientation.x = 0.1;
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);

  cloud = make_cloud();
  cloud.particles.front().pose.orientation.w = 2.0;
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);

  cloud = make_cloud();
  cloud.particles.front().pose.position.x =
    std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(cloud),
    std::invalid_argument);
}

TEST(AmclParticleCloudAdapter, RejectsInvalidConfiguration)
{
  auto config = hlr::AmclParticleCloudAdapterConfig{};
  config.expected_frame_id.clear();
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(make_cloud(), config),
    std::invalid_argument);

  config = {};
  config.quaternion_norm_tolerance = -1.0;
  EXPECT_THROW(
    (void)hlr::adapt_amcl_particle_cloud(make_cloud(), config),
    std::invalid_argument);
}

TEST(AmclParticleCloudAdapter, UsesSensorDataCompatibleQos)
{
  const auto profile = hlr::amcl_particle_cloud_qos().get_rmw_qos_profile();
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
