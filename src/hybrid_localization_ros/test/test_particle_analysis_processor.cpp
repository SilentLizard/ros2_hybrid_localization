#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "hybrid_localization_core/types.hpp"
#include "hybrid_localization_ros/particle_analysis_processor.hpp"

namespace hl = hybrid_localization;
namespace hlr = hybrid_localization_ros;

namespace
{

hlr::AdaptedParticleCloud two_cluster_cloud()
{
  hlr::AdaptedParticleCloud cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = 42;
  cloud.header.stamp.nanosec = 250000000U;

  // Two compact, well-separated modes. Seven particles in the dominant mode
  // and five in the secondary mode satisfy the default weighted DBSCAN
  // minimum-neighbor requirement while creating an intentionally ambiguous GMM.
  for (std::size_t index = 0U; index < 7U; ++index) {
    cloud.particles.push_back({
      .pose = {
        .x = 0.01 * static_cast<double>(index),
        .y = 0.0,
        .yaw = 0.01 * static_cast<double>(index),
      },
      .weight = 1.0 / 12.0,
    });
  }
  for (std::size_t index = 0U; index < 5U; ++index) {
    cloud.particles.push_back({
      .pose = {
        .x = 2.0 + 0.01 * static_cast<double>(index),
        .y = 1.0,
        .yaw = 1.0 + 0.01 * static_cast<double>(index),
      },
      .weight = 1.0 / 12.0,
    });
  }
  cloud.source_weight_sum = 1.0;
  return cloud;
}

}  // namespace

TEST(ParticleAnalysisProcessor, ProducesCoherentObservationProducts)
{
  hlr::ParticleAnalysisProcessor processor;
  const auto result = processor.process(two_cluster_cloud());

  EXPECT_EQ(result.analysis.header.frame_id, "map");
  EXPECT_EQ(result.analysis.header.stamp.sec, 42);
  EXPECT_EQ(result.analysis.header.stamp.nanosec, 250000000U);
  EXPECT_EQ(result.analysis.analysis_sequence, 1U);
  EXPECT_EQ(result.analysis.particle_count, 12U);
  EXPECT_EQ(result.analysis.retained_cluster_count, 2U);
  EXPECT_NEAR(result.analysis.effective_sample_size, 12.0, 1e-12);
  EXPECT_NEAR(result.analysis.retained_cluster_weight, 1.0, 1e-12);
  EXPECT_NEAR(result.analysis.noise_weight, 0.0, 1e-12);
  EXPECT_NEAR(result.analysis.dominant_cluster_weight, 7.0 / 12.0, 1e-12);

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_EQ(result.mixture.header.frame_id, "map");
  EXPECT_EQ(result.mixture.update_sequence, 1U);
  EXPECT_NEAR(result.mixture.discarded_weight, 0.0, 1e-12);
  EXPECT_NEAR(result.mixture.components[0].weight, 7.0 / 12.0, 1e-12);
  EXPECT_NEAR(result.mixture.components[1].weight, 5.0 / 12.0, 1e-12);
  EXPECT_NE(result.mixture.components[0].provenance.id, 0U);
  EXPECT_NE(result.mixture.components[1].provenance.id, 0U);

  EXPECT_EQ(result.health.update_sequence, 1U);
  EXPECT_EQ(result.health.component_count, 2U);
  EXPECT_NEAR(result.health.represented_weight, 1.0, 1e-12);
  EXPECT_FALSE(result.health.has_measurement_update);
  EXPECT_FALSE(result.health.has_fit_quality);
  EXPECT_FALSE(result.health.has_recovery_failure_score);

  EXPECT_TRUE(result.evidence.particle_belief_converged);
  EXPECT_TRUE(result.evidence.gmm_available);
  EXPECT_TRUE(result.evidence.tracking_ambiguous);

  // The aggregate embeds the same products published individually.
  EXPECT_EQ(result.analysis.mixture.update_sequence, result.mixture.update_sequence);
  EXPECT_EQ(result.analysis.health.update_sequence, result.health.update_sequence);
  EXPECT_EQ(
    result.analysis.evidence.tracking_ambiguous,
    result.evidence.tracking_ambiguous);
}

TEST(ParticleAnalysisProcessor, SequenceAndRootHypothesisIdsAdvanceAcrossClouds)
{
  hlr::ParticleAnalysisProcessor processor;
  const auto cloud = two_cluster_cloud();

  const auto first = processor.process(cloud);
  const auto second = processor.process(cloud);

  EXPECT_EQ(first.analysis.analysis_sequence, 1U);
  EXPECT_EQ(second.analysis.analysis_sequence, 2U);
  EXPECT_EQ(processor.analysis_sequence(), 2U);

  ASSERT_EQ(first.mixture.components.size(), 2U);
  ASSERT_EQ(second.mixture.components.size(), 2U);
  EXPECT_LT(
    first.mixture.components[0].provenance.id,
    second.mixture.components[0].provenance.id);
  EXPECT_LT(
    first.mixture.components[1].provenance.id,
    second.mixture.components[1].provenance.id);
}

TEST(ParticleAnalysisProcessor, RejectsInvalidSourceTimestampTransactionally)
{
  hlr::ParticleAnalysisProcessor processor;
  auto cloud = two_cluster_cloud();
  cloud.header.stamp.sec = -1;
  cloud.header.stamp.nanosec = 0U;

  EXPECT_THROW((void)processor.process(cloud), std::invalid_argument);
  EXPECT_EQ(processor.analysis_sequence(), 0U);

  cloud.header.stamp.sec = 1;
  const auto valid = processor.process(cloud);
  EXPECT_EQ(valid.analysis.analysis_sequence, 1U);
  ASSERT_FALSE(valid.mixture.components.empty());
  EXPECT_EQ(valid.mixture.components.front().provenance.id, 1U);
}

TEST(ParticleAnalysisProcessor, RejectsEmptyParticlePopulation)
{
  hlr::ParticleAnalysisProcessor processor;
  hlr::AdaptedParticleCloud cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = 1;

  EXPECT_THROW((void)processor.process(cloud), std::invalid_argument);
  EXPECT_EQ(processor.analysis_sequence(), 0U);
}

TEST(ParticleAnalysisProcessor, ValidatesConfigurationAtConstruction)
{
  hlr::ParticleAnalysisProcessorConfig config;
  config.clustering.position_scale = 0.0;

  EXPECT_THROW((void)hlr::ParticleAnalysisProcessor{config}, std::invalid_argument);
}
