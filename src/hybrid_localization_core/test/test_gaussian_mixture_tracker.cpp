#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <vector>

#include "hybrid_localization_core/gaussian_mixture_tracker.hpp"
#include "hybrid_localization_core/hypothesis_provenance.hpp"

namespace hybrid_localization
{
namespace
{

[[nodiscard]] GaussianComponent component(
  HypothesisIdGenerator & ids,
  const double x,
  const double weight,
  const double variance = 0.1)
{
  GaussianComponent value;
  value.mean = Pose2d{x, 0.0, 0.0};
  value.covariance = {
    variance, 0.0, 0.0,
    0.0, variance, 0.0,
    0.0, 0.0, variance};
  value.weight = weight;
  value.sample_count = 4U;
  value.provenance = make_root_provenance(ids);
  return value;
}

[[nodiscard]] GaussianMotionIncrement stationary_motion()
{
  return GaussianMotionIncrement{};
}

TEST(GaussianMixtureTracker, PreservesIdentityAcrossRecursiveUpdate)
{
  HypothesisIdGenerator ids;
  GaussianMixture initial{{component(ids, 0.0, 1.0)}, 0.0};
  const auto original_id = initial.components.front().provenance.id;

  GaussianMixtureTrackerConfig config;
  config.maximum_component_count = 4U;
  config.enable_splitting = false;

  GaussianMixtureTracker tracker(initial, config);
  const auto result = tracker.update(GaussianMixtureTrackerInput{stationary_motion()});

  ASSERT_EQ(result.mixture.components.size(), 1U);
  EXPECT_EQ(result.mixture.components.front().provenance.id, original_id);
  EXPECT_EQ(result.update_sequence, 1U);
  EXPECT_EQ(tracker.update_sequence(), 1U);
  EXPECT_TRUE(result.component_limit_enforced);
}

TEST(GaussianMixtureTracker, AppliesObservationAndReportsHealth)
{
  HypothesisIdGenerator ids;
  GaussianMixture initial{{component(ids, 0.0, 1.0)}, 0.0};

  GaussianMixtureTrackerConfig config;
  config.maximum_component_count = 2U;
  config.enable_splitting = false;

  GaussianMixtureTracker tracker(initial, config);
  GaussianPoseObservation observation;
  observation.mean = Pose2d{0.2, 0.0, 0.0};
  observation.covariance = {
    0.1, 0.0, 0.0,
    0.0, 0.1, 0.0,
    0.0, 0.0, 0.1};

  GaussianMixtureTrackerInput input;
  input.motion = stationary_motion();
  input.observation = observation;
  const auto result = tracker.update(input);

  ASSERT_TRUE(result.measurement_update.has_value());
  EXPECT_TRUE(result.measurement_applied);
  EXPECT_TRUE(result.pre_topology_health.has_measurement_update);
  EXPECT_EQ(result.final_health.component_count, 1U);
  EXPECT_GT(result.mixture.components.front().mean.x, 0.0);
}

TEST(GaussianMixtureTracker, MergeCreatesTrackedDescendant)
{
  HypothesisIdGenerator ids;
  const auto first = component(ids, 0.0, 0.5);
  const auto second = component(ids, 0.1, 0.5);
  GaussianMixture initial{{first, second}, 0.0};

  GaussianMixtureTrackerConfig config;
  config.maximum_component_count = 4U;
  config.enable_splitting = false;
  config.merging.maximum_mahalanobis_distance_squared = 100.0;
  config.merging.maximum_yaw_difference = 1.0;

  GaussianMixtureTracker tracker(initial, config);
  const auto result = tracker.update(GaussianMixtureTrackerInput{stationary_motion()});

  ASSERT_EQ(result.merging.merge_count, 1U);
  ASSERT_EQ(result.mixture.components.size(), 1U);
  ASSERT_EQ(result.merging.created_hypothesis_ids.size(), 1U);
  const auto & provenance = result.mixture.components.front().provenance;
  EXPECT_EQ(provenance.id, result.merging.created_hypothesis_ids.front());
  EXPECT_EQ(provenance.parent_count, 2U);
  EXPECT_EQ(provenance.event, HypothesisProvenanceEvent::merge);
}

TEST(GaussianMixtureTracker, SplitsByStableHypothesisId)
{
  HypothesisIdGenerator ids;
  auto parent = component(ids, 0.0, 1.0, 2.0);
  const auto parent_id = parent.provenance.id;
  GaussianMixture initial{{parent}, 0.0};

  const std::vector<WeightedParticle> particles{
    {{-2.2, 0.0, 0.0}, 1.0},
    {{-2.0, 0.0, 0.0}, 1.0},
    {{-1.8, 0.0, 0.0}, 1.0},
    {{1.8, 0.0, 0.0}, 1.0},
    {{2.0, 0.0, 0.0}, 1.0},
    {{2.2, 0.0, 0.0}, 1.0}};
  const std::array<std::size_t, 6> indices{0U, 1U, 2U, 3U, 4U, 5U};

  GaussianTrackerSplitEvidence evidence;
  evidence.hypothesis_id = parent_id;
  evidence.source_indices = indices;
  evidence.fit_quality = GaussianFitQuality{2.0, 3.0, 1.0};

  GaussianMixtureTrackerConfig config;
  config.maximum_component_count = 4U;
  config.merging.maximum_mahalanobis_distance_squared = 0.0;
  config.merging.maximum_yaw_difference = 0.0;
  config.splitting.maximum_mean_mahalanobis_distance = 1.0;
  config.splitting.minimum_source_samples = 4U;
  config.splitting.minimum_child_samples = 2U;
  config.splitting.maximum_splits = 1U;

  GaussianMixtureTracker tracker(initial, config);
  GaussianMixtureTrackerInput input;
  input.motion = stationary_motion();
  input.source_particles = particles;
  input.split_evidence = std::span<const GaussianTrackerSplitEvidence>(&evidence, 1U);

  const auto result = tracker.update(input);

  ASSERT_TRUE(result.splitting.has_value());
  EXPECT_EQ(result.splitting->split_count, 1U);
  ASSERT_EQ(result.mixture.components.size(), 2U);
  for (const auto & child : result.mixture.components) {
    EXPECT_EQ(child.provenance.parent_count, 1U);
    EXPECT_EQ(child.provenance.parent_ids[0], parent_id);
    EXPECT_EQ(child.provenance.event, HypothesisProvenanceEvent::split);
  }
}

TEST(GaussianMixtureTracker, EnforcesHardComponentLimitAfterTopologyChanges)
{
  HypothesisIdGenerator ids;
  GaussianMixture initial{{
    component(ids, 0.0, 0.4),
    component(ids, 5.0, 0.3),
    component(ids, 10.0, 0.2),
    component(ids, 15.0, 0.1)}, 0.0};

  GaussianMixtureTrackerConfig config;
  config.maximum_component_count = 2U;
  config.enable_splitting = false;

  GaussianMixtureTracker tracker(initial, config);
  EXPECT_EQ(tracker.mixture().components.size(), 2U);

  const auto result = tracker.update(GaussianMixtureTrackerInput{stationary_motion()});
  EXPECT_LE(result.mixture.components.size(), 2U);
  EXPECT_TRUE(result.component_limit_enforced);
}

TEST(GaussianMixtureTracker, RejectsDuplicateHypothesisIds)
{
  HypothesisIdGenerator ids;
  const auto first = component(ids, 0.0, 0.5);
  auto second = component(ids, 1.0, 0.5);
  second.provenance = first.provenance;

  const GaussianMixture invalid{{first, second}, 0.0};
  EXPECT_THROW(
    GaussianMixtureTracker(invalid, GaussianMixtureTrackerConfig{}),
    std::invalid_argument);
}

TEST(GaussianMixtureTracker, ResetRestartsSequenceAndAdvancesIdsPastMixture)
{
  HypothesisIdGenerator ids;
  GaussianMixture initial{{component(ids, 0.0, 1.0)}, 0.0};
  GaussianMixtureTracker tracker(initial);

  static_cast<void>(tracker.update(GaussianMixtureTrackerInput{stationary_motion()}));
  EXPECT_EQ(tracker.update_sequence(), 1U);

  HypothesisIdGenerator reset_ids(20U);
  GaussianMixture replacement{{component(reset_ids, 3.0, 1.0)}, 0.0};
  tracker.reset(replacement);

  EXPECT_EQ(tracker.update_sequence(), 0U);
  EXPECT_EQ(tracker.next_hypothesis_id(), 21U);
}

}  // namespace
}  // namespace hybrid_localization
