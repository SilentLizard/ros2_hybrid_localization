#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_localization_core/gaussian_fit_quality.hpp"
#include "hybrid_localization_core/gaussian_statistics.hpp"

namespace hl = hybrid_localization;

TEST(GaussianFitQuality, EvaluatesSymmetricOneDimensionalFit)
{
  const std::vector<hl::WeightedParticle> particles{
    {{-1.0, 0.0, 0.0}, 0.5},
    {{1.0, 0.0, 0.0}, 0.5}
  };
  const std::vector<std::size_t> indices{0U, 1U};
  const hl::GaussianComponent component = hl::fit_gaussian(particles, indices);

  const auto quality = hl::evaluate_gaussian_fit_quality(
    particles,
    indices,
    component,
    {1e-12});

  EXPECT_NEAR(quality.mean_mahalanobis_distance, 1.0, 1e-9);
  EXPECT_NEAR(quality.maximum_mahalanobis_distance, 1.0, 1e-9);
  EXPECT_NEAR(quality.angular_resultant_length, 1.0, 1e-12);
}

TEST(GaussianFitQuality, UsesConditionalParticleWeights)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.75},
    {{2.0, 0.0, 0.0}, 0.25}
  };
  const std::vector<std::size_t> indices{0U, 1U};
  const hl::GaussianComponent component = hl::fit_gaussian(particles, indices);

  const auto quality = hl::evaluate_gaussian_fit_quality(
    particles,
    indices,
    component,
    {1e-12});

  EXPECT_NEAR(
    quality.mean_mahalanobis_distance,
    std::sqrt(3.0) / 2.0,
    1e-9);
  EXPECT_NEAR(
    quality.maximum_mahalanobis_distance,
    std::sqrt(3.0),
    1e-9);
}

TEST(GaussianFitQuality, ReportsWrappedYawConcentration)
{
  const double one_degree = std::numbers::pi / 180.0;
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, std::numbers::pi - one_degree}, 0.5},
    {{0.0, 0.0, -std::numbers::pi + one_degree}, 0.5}
  };
  const std::vector<std::size_t> indices{0U, 1U};
  const hl::GaussianComponent component = hl::fit_gaussian(particles, indices);

  const auto quality = hl::evaluate_gaussian_fit_quality(
    particles,
    indices,
    component);

  EXPECT_NEAR(
    quality.angular_resultant_length,
    std::cos(one_degree),
    1e-12);
  EXPECT_LT(quality.mean_mahalanobis_distance, 1.1);
  EXPECT_LT(quality.maximum_mahalanobis_distance, 1.1);
}

TEST(GaussianFitQuality, MaximumDistanceExposesOutlier)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.45},
    {{0.1, 0.0, 0.0}, 0.45},
    {{10.0, 0.0, 0.0}, 0.10}
  };
  const std::vector<std::size_t> indices{0U, 1U, 2U};
  const hl::GaussianComponent component = hl::fit_gaussian(particles, indices);

  const auto quality = hl::evaluate_gaussian_fit_quality(
    particles,
    indices,
    component);

  EXPECT_GT(
    quality.maximum_mahalanobis_distance,
    quality.mean_mahalanobis_distance);
  EXPECT_GT(quality.maximum_mahalanobis_distance, 2.0);
}

TEST(GaussianFitQuality, SupportsSingularFittedCovarianceWithRegularization)
{
  const std::vector<hl::WeightedParticle> particles{
    {{1.0, 2.0, 0.5}, 1.0}
  };
  const std::vector<std::size_t> indices{0U};
  const hl::GaussianComponent component = hl::fit_gaussian(particles, indices);

  const auto quality = hl::evaluate_gaussian_fit_quality(
    particles,
    indices,
    component);

  EXPECT_DOUBLE_EQ(quality.mean_mahalanobis_distance, 0.0);
  EXPECT_DOUBLE_EQ(quality.maximum_mahalanobis_distance, 0.0);
  EXPECT_NEAR(quality.angular_resultant_length, 1.0, 1e-12);
}

TEST(GaussianFitQuality, RejectsInvalidInput)
{
  const std::vector<hl::WeightedParticle> particles{
    {{0.0, 0.0, 0.0}, 0.5},
    {{1.0, 0.0, 0.0}, 0.5}
  };
  const std::vector<std::size_t> indices{0U, 1U};
  const hl::GaussianComponent component = hl::fit_gaussian(particles, indices);

  EXPECT_THROW(
    static_cast<void>(hl::evaluate_gaussian_fit_quality(
      particles,
      {},
      component)),
    std::invalid_argument);

  const std::vector<std::size_t> duplicate_indices{0U, 0U};
  EXPECT_THROW(
    static_cast<void>(hl::evaluate_gaussian_fit_quality(
      particles,
      duplicate_indices,
      component)),
    std::invalid_argument);

  const std::vector<std::size_t> out_of_range_indices{0U, 2U};
  EXPECT_THROW(
    static_cast<void>(hl::evaluate_gaussian_fit_quality(
      particles,
      out_of_range_indices,
      component)),
    std::invalid_argument);

  EXPECT_THROW(
    static_cast<void>(hl::evaluate_gaussian_fit_quality(
      particles,
      indices,
      component,
      {0.0})),
    std::invalid_argument);

  hl::GaussianComponent asymmetric = component;
  asymmetric.covariance[1] = 1.0;
  EXPECT_THROW(
    static_cast<void>(hl::evaluate_gaussian_fit_quality(
      particles,
      indices,
      asymmetric)),
    std::invalid_argument);

  hl::GaussianComponent non_finite = component;
  non_finite.mean.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    static_cast<void>(hl::evaluate_gaussian_fit_quality(
      particles,
      indices,
      non_finite)),
    std::invalid_argument);
}