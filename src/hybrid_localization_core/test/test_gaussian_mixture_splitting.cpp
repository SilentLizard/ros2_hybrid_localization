#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_localization_core/gaussian_fit_quality.hpp"
#include "hybrid_localization_core/gaussian_mixture_splitting.hpp"
#include "hybrid_localization_core/gaussian_statistics.hpp"

namespace hl = hybrid_localization;

namespace
{

std::vector<hl::WeightedParticle> bimodal_particles()
{
  return {
    {{-2.1, 0.0, 0.0}, 0.15},
    {{-1.9, 0.0, 0.0}, 0.15},
    {{-2.0, 0.1, 0.0}, 0.20},
    {{1.9, 0.0, 0.0}, 0.15},
    {{2.1, 0.0, 0.0}, 0.15},
    {{2.0, -0.1, 0.0}, 0.20}};
}

hl::GaussianComponentSplitEvidence make_evidence(
  const std::vector<hl::WeightedParticle> & particles,
  const std::vector<std::size_t> & indices,
  const hl::GaussianComponent & component)
{
  return {
    indices,
    hl::evaluate_gaussian_fit_quality(particles, indices, component)};
}

hl::GaussianMixtureSplittingConfig split_config()
{
  hl::GaussianMixtureSplittingConfig config;
  config.maximum_mean_mahalanobis_distance = 0.0;
  config.maximum_fit_mahalanobis_distance = 0.0;
  config.minimum_source_samples = 4U;
  config.minimum_child_samples = 2U;
  config.minimum_child_weight_fraction = 0.2;
  config.maximum_splits = 1U;
  return config;
}

}  // namespace

TEST(GaussianMixtureSplitting, SplitsBimodalComponentAndPreservesMass)
{
  const auto particles = bimodal_particles();
  const std::vector<std::size_t> indices{0U, 1U, 2U, 3U, 4U, 5U};
  const auto parent = hl::fit_gaussian(particles, indices);
  const hl::GaussianMixture mixture{{parent}, 0.0};
  const std::vector<hl::GaussianComponentSplitEvidence> evidence{
    make_evidence(particles, indices, parent)};

  const auto result = hl::split_gaussian_mixture_components(
    particles, mixture, evidence, split_config());

  ASSERT_EQ(result.split_count, 1U);
  ASSERT_EQ(result.split_component_indices,
    std::vector<std::size_t>({0U}));
  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_NEAR(
    result.mixture.components[0].weight +
    result.mixture.components[1].weight,
    1.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.mixture.discarded_weight, 0.0);
  EXPECT_LT(result.mixture.components[0].mean.x, -1.5);
  EXPECT_GT(result.mixture.components[1].mean.x, 1.5);
}

TEST(GaussianMixtureSplitting, RescalesChildWeightsToParentMass)
{
  auto particles = bimodal_particles();
  for (auto & particle : particles) {
    particle.weight *= 0.6;
  }
  particles.push_back({{10.0, 0.0, 0.0}, 0.4});

  const std::vector<std::size_t> indices{0U, 1U, 2U, 3U, 4U, 5U};
  auto parent = hl::fit_gaussian(particles, indices);
  parent.weight = 0.7;
  const hl::GaussianMixture mixture{{parent}, 0.3};
  const std::vector<hl::GaussianComponentSplitEvidence> evidence{
    make_evidence(particles, indices, parent)};

  const auto result = hl::split_gaussian_mixture_components(
    particles, mixture, evidence, split_config());

  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_NEAR(
    result.mixture.components[0].weight +
    result.mixture.components[1].weight,
    0.7, 1e-12);
  EXPECT_NEAR(result.mixture.discarded_weight, 0.3, 1e-12);
}

TEST(GaussianMixtureSplitting, LeavesGoodFitComponentUnchanged)
{
  const auto particles = bimodal_particles();
  const std::vector<std::size_t> indices{0U, 1U, 2U, 3U, 4U, 5U};
  const auto parent = hl::fit_gaussian(particles, indices);
  const hl::GaussianMixture mixture{{parent}, 0.0};

  hl::GaussianComponentSplitEvidence evidence{
    indices, {0.1, 0.2, 1.0}};
  auto config = split_config();
  config.maximum_mean_mahalanobis_distance = 1.0;
  config.maximum_fit_mahalanobis_distance = 2.0;

  const auto result = hl::split_gaussian_mixture_components(
    particles, mixture,
    std::span<const hl::GaussianComponentSplitEvidence>(&evidence, 1U),
    config);

  ASSERT_EQ(result.split_count, 0U);
  ASSERT_EQ(result.mixture.components.size(), 1U);
  EXPECT_NEAR(result.mixture.components[0].mean.x, parent.mean.x, 1e-12);
  EXPECT_NEAR(result.mixture.components[0].weight, parent.weight, 1e-12);
}

TEST(GaussianMixtureSplitting, UsesAngularEvidence)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, -1.2}, 0.25},
    {{0.0, 0.0, -1.0}, 0.25},
    {{0.0, 0.0, 1.0}, 0.25},
    {{0.0, 0.0, 1.2}, 0.25}};
  const std::vector<std::size_t> indices{0U, 1U, 2U, 3U};
  const auto parent = hl::fit_gaussian(particles, indices);
  const hl::GaussianMixture mixture{{parent}, 0.0};
  const std::vector<hl::GaussianComponentSplitEvidence> evidence{
    make_evidence(particles, indices, parent)};

  auto config = split_config();
  config.maximum_mean_mahalanobis_distance =
    std::numeric_limits<double>::infinity();
  config.maximum_fit_mahalanobis_distance =
    std::numeric_limits<double>::infinity();
  config.minimum_angular_resultant_length = 0.8;

  const auto result = hl::split_gaussian_mixture_components(
    particles, mixture, evidence, config);

  ASSERT_EQ(result.split_count, 1U);
  ASSERT_EQ(result.mixture.components.size(), 2U);
  EXPECT_LT(result.mixture.components[0].mean.yaw, -0.8);
  EXPECT_GT(result.mixture.components[1].mean.yaw, 0.8);
}

TEST(GaussianMixtureSplitting, RespectsMaximumSplitCountAndStableOrder)
{
  const auto first_particles = bimodal_particles();
  std::vector<hl::WeightedParticle> particles = first_particles;
  for (const auto & particle : first_particles) {
    auto shifted = particle;
    shifted.pose.y += 10.0;
    particles.push_back(shifted);
  }
  for (auto & particle : particles) {
    particle.weight *= 0.5;
  }

  const std::vector<std::size_t> first{0U, 1U, 2U, 3U, 4U, 5U};
  const std::vector<std::size_t> second{6U, 7U, 8U, 9U, 10U, 11U};
  const auto first_component = hl::fit_gaussian(particles, first);
  const auto second_component = hl::fit_gaussian(particles, second);
  const hl::GaussianMixture mixture{
    {first_component, second_component}, 0.0};
  const std::vector<hl::GaussianComponentSplitEvidence> evidence{
    make_evidence(particles, first, first_component),
    make_evidence(particles, second, second_component)};

  auto config = split_config();
  config.maximum_splits = 1U;
  const auto result = hl::split_gaussian_mixture_components(
    particles, mixture, evidence, config);

  ASSERT_EQ(result.split_count, 1U);
  ASSERT_EQ(result.split_component_indices,
    std::vector<std::size_t>({0U}));
  ASSERT_EQ(result.mixture.components.size(), 3U);
}

TEST(GaussianMixtureSplitting, RejectsUnsplittableChildConstraints)
{
  // Any valid boundary must leave at least two particles on each side.
  // The first two particles already carry 90% of the source mass, so no
  // feasible boundary can give both children at least 40%.
  const std::vector<hl::WeightedParticle> particles{
    {{-2.1, 0.0, 0.0}, 0.45},
    {{-1.9, 0.0, 0.0}, 0.45},
    {{1.8, 0.0, 0.0}, 0.025},
    {{1.9, 0.0, 0.0}, 0.025},
    {{2.0, 0.0, 0.0}, 0.025},
    {{2.1, 0.0, 0.0}, 0.025}};

  const std::vector<std::size_t> indices{
    0U, 1U, 2U, 3U, 4U, 5U};

  const auto parent = hl::fit_gaussian(particles, indices);
  const hl::GaussianMixture mixture{{parent}, 0.0};

  const std::vector<hl::GaussianComponentSplitEvidence> evidence{
    make_evidence(particles, indices, parent)};

  auto config = split_config();
  config.minimum_child_weight_fraction = 0.4;

  EXPECT_THROW(
    static_cast<void>(
      hl::split_gaussian_mixture_components(
        particles, mixture, evidence, config)),
    std::domain_error);
}

TEST(GaussianMixtureSplitting, RejectsInvalidInput)
{
  const auto particles = bimodal_particles();
  const std::vector<std::size_t> indices{0U, 1U, 2U, 3U, 4U, 5U};
  const auto parent = hl::fit_gaussian(particles, indices);
  const hl::GaussianMixture mixture{{parent}, 0.0};
  const std::vector<hl::GaussianComponentSplitEvidence> evidence{
    make_evidence(particles, indices, parent)};

  EXPECT_THROW(
    static_cast<void>(hl::split_gaussian_mixture_components(
      particles, mixture, {}, split_config())),
    std::invalid_argument);

  auto invalid_config = split_config();
  invalid_config.minimum_source_samples = 3U;
  invalid_config.minimum_child_samples = 2U;
  EXPECT_THROW(
    static_cast<void>(hl::split_gaussian_mixture_components(
      particles, mixture, evidence, invalid_config)),
    std::invalid_argument);

  invalid_config = split_config();
  invalid_config.minimum_child_weight_fraction = 0.0;
  EXPECT_THROW(
    static_cast<void>(hl::split_gaussian_mixture_components(
      particles, mixture, evidence, invalid_config)),
    std::invalid_argument);

  auto invalid_mixture = mixture;
  invalid_mixture.discarded_weight = 0.2;
  EXPECT_THROW(
    static_cast<void>(hl::split_gaussian_mixture_components(
      particles, invalid_mixture, evidence, split_config())),
    std::invalid_argument);

  const std::vector<std::size_t> duplicate_indices{0U, 0U, 1U, 2U};
  const std::vector<hl::GaussianComponentSplitEvidence> duplicate_evidence{
    {duplicate_indices, evidence[0].fit_quality}};
  EXPECT_THROW(
    static_cast<void>(hl::split_gaussian_mixture_components(
      particles, mixture, duplicate_evidence, split_config())),
    std::invalid_argument);
}
