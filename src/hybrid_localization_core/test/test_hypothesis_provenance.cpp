#include "hybrid_localization_core/hypothesis_provenance.hpp"
#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/gaussian_mixture_management.hpp"
#include "hybrid_localization_core/gaussian_mixture_merging.hpp"
#include "hybrid_localization_core/gaussian_mixture_prediction.hpp"

#include <gtest/gtest.h>

namespace hl = hybrid_localization;

TEST(HypothesisProvenance, AllocatesMonotonicNonzeroIds)
{
  hl::HypothesisIdGenerator generator(41U);
  EXPECT_EQ(generator.next(), 41U);
  EXPECT_EQ(generator.next(), 42U);
  EXPECT_EQ(generator.peek_next(), 43U);
}

TEST(HypothesisProvenance, BuildsRootMergeAndSplitLineage)
{
  hl::HypothesisIdGenerator generator(10U);
  const auto first = hl::make_root_provenance(generator);
  const auto second = hl::make_root_provenance(generator);
  const auto merged = hl::make_merged_provenance(generator, first, second);
  const auto child = hl::make_split_provenance(generator, merged);

  EXPECT_EQ(first.id, 10U);
  EXPECT_EQ(first.generation, 0U);
  EXPECT_EQ(merged.id, 12U);
  EXPECT_EQ(merged.parent_count, 2U);
  EXPECT_EQ(merged.parent_ids[0], first.id);
  EXPECT_EQ(merged.parent_ids[1], second.id);
  EXPECT_EQ(merged.generation, 1U);
  EXPECT_EQ(merged.event, hl::HypothesisProvenanceEvent::merge);
  EXPECT_EQ(child.parent_count, 1U);
  EXPECT_EQ(child.parent_ids[0], merged.id);
  EXPECT_EQ(child.generation, 2U);
  EXPECT_EQ(child.event, hl::HypothesisProvenanceEvent::split);
}

TEST(HypothesisProvenance, RejectsInconsistentMetadata)
{
  hl::HypothesisProvenance invalid;
  invalid.id = 3U;
  EXPECT_THROW(
    hl::validate_hypothesis_provenance(invalid),
    std::invalid_argument);

  invalid.event = hl::HypothesisProvenanceEvent::merge;
  invalid.parent_count = 1U;
  invalid.parent_ids[0] = invalid.id;
  invalid.generation = 1U;
  EXPECT_THROW(
    hl::validate_hypothesis_provenance(invalid),
    std::invalid_argument);
}

TEST(HypothesisProvenance, PredictionPreservesIdentity)
{
  hl::HypothesisIdGenerator generator;
  hl::GaussianComponent component;
  component.weight = 1.0;
  component.covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  component.provenance = hl::make_root_provenance(generator);

  hl::GaussianMotionIncrement motion;
  motion.covariance = {};
  const auto predicted = hl::predict_gaussian_component(component, motion);
  EXPECT_EQ(predicted.provenance.id, component.provenance.id);
  EXPECT_EQ(predicted.provenance.generation, component.provenance.generation);
  EXPECT_EQ(predicted.provenance.event, component.provenance.event);
}

TEST(HypothesisProvenance, ManagementReportsPrunedIds)
{
  hl::HypothesisIdGenerator generator;
  hl::GaussianMixture mixture;
  mixture.components.resize(2);
  mixture.components[0].weight = 0.9;
  mixture.components[1].weight = 0.1;
  mixture.components[0].provenance = hl::make_root_provenance(generator);
  mixture.components[1].provenance = hl::make_root_provenance(generator);

  hl::GaussianMixtureManagementConfig config;
  config.minimum_component_weight = 0.2;
  const auto result = hl::manage_gaussian_mixture(mixture, config);

  ASSERT_EQ(result.pruned_hypothesis_ids.size(), 1U);
  EXPECT_EQ(result.pruned_hypothesis_ids[0], mixture.components[1].provenance.id);
  ASSERT_EQ(result.mixture.components.size(), 1U);
  EXPECT_EQ(result.mixture.components[0].provenance.id,
    mixture.components[0].provenance.id);
}

TEST(HypothesisProvenance, MergeAllocatesNewIdAndParents)
{
  hl::HypothesisIdGenerator generator(100U);
  hl::GaussianMixture mixture;
  mixture.components.resize(2);
  for (auto & component : mixture.components) {
    component.weight = 0.5;
    component.covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    component.provenance = hl::make_root_provenance(generator);
  }

  hl::GaussianMixtureMergingConfig config;
  config.maximum_mahalanobis_distance_squared = 1.0;
  config.maximum_yaw_difference = 1.0;
  const auto result = hl::merge_gaussian_mixture_components(
    mixture, generator, config);

  ASSERT_EQ(result.mixture.components.size(), 1U);
  ASSERT_EQ(result.created_hypothesis_ids.size(), 1U);
  const auto & provenance = result.mixture.components[0].provenance;
  EXPECT_EQ(provenance.id, result.created_hypothesis_ids[0]);
  EXPECT_EQ(provenance.parent_count, 2U);
  EXPECT_EQ(provenance.parent_ids[0], mixture.components[0].provenance.id);
  EXPECT_EQ(provenance.parent_ids[1], mixture.components[1].provenance.id);
}
