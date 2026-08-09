#include <cstdint>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "hybrid_localization_ros/particle_analysis_parameters.hpp"

namespace hlr = hybrid_localization_ros;

TEST(ParticleAnalysisParameters, AppliesClusteringAndHealthPolicyUpdates)
{
  hlr::ParticleAnalysisProcessorConfig current;
  const std::vector<rclcpp::Parameter> updates{
    rclcpp::Parameter("particle_clustering.epsilon", 0.6),
    rclcpp::Parameter("particle_clustering.minimum_neighbors", std::int64_t{7}),
    rclcpp::Parameter("health_policy.particle_minimum_dominant_weight", 0.65),
    rclcpp::Parameter("health_policy.maximum_component_count", std::int64_t{6}),
  };

  const auto candidate = hlr::apply_particle_analysis_parameter_updates(current, updates);

  EXPECT_DOUBLE_EQ(candidate.clustering.epsilon, 0.6);
  EXPECT_EQ(candidate.clustering.minimum_neighbors, 7U);
  EXPECT_DOUBLE_EQ(candidate.evidence_policy.particle_minimum_dominant_weight, 0.65);
  EXPECT_EQ(candidate.evidence_policy.maximum_component_count, 6U);
}

TEST(ParticleAnalysisParameters, RejectsInvalidClusteringTransaction)
{
  hlr::ParticleAnalysisProcessorConfig current;
  const std::vector<rclcpp::Parameter> updates{
    rclcpp::Parameter("particle_clustering.position_scale", 0.0),
    rclcpp::Parameter("particle_clustering.minimum_neighbors", std::int64_t{7}),
  };

  EXPECT_THROW(
    (void)hlr::apply_particle_analysis_parameter_updates(current, updates),
    std::invalid_argument);

  EXPECT_DOUBLE_EQ(current.clustering.position_scale, 0.25);
  EXPECT_EQ(current.clustering.minimum_neighbors, 5U);
}

TEST(ParticleAnalysisParameters, RejectsInvalidCrossThresholdOrdering)
{
  hlr::ParticleAnalysisProcessorConfig current;
  const std::vector<rclcpp::Parameter> updates{
    rclcpp::Parameter("health_policy.good_minimum_represented_weight", 0.40),
    rclcpp::Parameter("health_policy.bad_maximum_represented_weight", 0.60),
  };

  EXPECT_THROW(
    (void)hlr::apply_particle_analysis_parameter_updates(current, updates),
    std::invalid_argument);
}

TEST(ParticleAnalysisParameters, RejectsNegativeCountBeforeUnsignedConversion)
{
  hlr::ParticleAnalysisProcessorConfig current;
  const std::vector<rclcpp::Parameter> updates{
    rclcpp::Parameter("particle_clustering.minimum_neighbors", std::int64_t{-1}),
  };

  EXPECT_THROW(
    (void)hlr::apply_particle_analysis_parameter_updates(current, updates),
    std::invalid_argument);
}

TEST(ParticleAnalysisParameters, IgnoresUnrelatedParameters)
{
  hlr::ParticleAnalysisProcessorConfig current;
  const std::vector<rclcpp::Parameter> updates{
    rclcpp::Parameter("use_sim_time", true),
  };

  const auto candidate = hlr::apply_particle_analysis_parameter_updates(current, updates);
  EXPECT_DOUBLE_EQ(candidate.clustering.position_scale, current.clustering.position_scale);
  EXPECT_DOUBLE_EQ(
    candidate.evidence_policy.good_minimum_represented_weight,
    current.evidence_policy.good_minimum_represented_weight);
  EXPECT_FALSE(hlr::is_particle_analysis_parameter("use_sim_time"));
  EXPECT_TRUE(hlr::is_particle_analysis_parameter("particle_clustering.epsilon"));
  EXPECT_TRUE(hlr::is_particle_analysis_parameter("health_policy.maximum_component_count"));
}
