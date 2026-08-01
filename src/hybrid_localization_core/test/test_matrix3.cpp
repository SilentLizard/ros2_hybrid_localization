#include "hybrid_localization_core/detail/matrix3.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

namespace hl = hybrid_localization;

TEST(Matrix3, ConvertsRowMajorRoundTrip)
{
  const hl::detail::Matrix3Storage values{
    1.0, 2.0, 3.0,
    4.0, 5.0, 6.0,
    7.0, 8.0, 9.0};
  EXPECT_EQ(
    hl::detail::matrix3_to_row_major(
      hl::detail::matrix3_from_row_major(values)),
    values);
}

TEST(Matrix3, SymmetrizesMatrix)
{
  hl::detail::Matrix3Storage values{
    1.0, 2.0, 4.0,
    0.0, 3.0, 6.0,
    2.0, 4.0, 5.0};
  hl::detail::symmetrize(values);
  EXPECT_DOUBLE_EQ(values[1], 1.0);
  EXPECT_DOUBLE_EQ(values[3], 1.0);
  EXPECT_DOUBLE_EQ(values[2], 3.0);
  EXPECT_DOUBLE_EQ(values[6], 3.0);
  EXPECT_DOUBLE_EQ(values[5], 5.0);
  EXPECT_DOUBLE_EQ(values[7], 5.0);
}

TEST(Matrix3, ValidatesPositiveSemidefiniteCovariance)
{
  const hl::detail::Matrix3Storage covariance{
    2.0, 0.5, 0.0,
    0.5, 1.0, 0.0,
    0.0, 0.0, 0.0};
  EXPECT_NO_THROW(
    hl::detail::validate_covariance(covariance, 1e-12, 1e-12, "covariance"));
}

TEST(Matrix3, RejectsAsymmetricAndIndefiniteCovariance)
{
  const hl::detail::Matrix3Storage asymmetric{
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  EXPECT_THROW(
    hl::detail::validate_covariance(asymmetric, 1e-12, 1e-12, "covariance"),
    std::invalid_argument);

  const hl::detail::Matrix3Storage indefinite{
    1.0, 2.0, 0.0,
    2.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  EXPECT_THROW(
    hl::detail::validate_covariance(indefinite, 1e-12, 1e-12, "covariance"),
    std::invalid_argument);
}

TEST(Matrix3, SolvesAndComputesMahalanobisDistance)
{
  const hl::detail::Matrix3Storage covariance{
    2.0, 0.0, 0.0,
    0.0, 4.0, 0.0,
    0.0, 0.0, 8.0};
  const hl::detail::Vector3Storage residual{2.0, 4.0, 8.0};
  const auto solved = hl::detail::solve_positive_definite(
    covariance, residual, "covariance");
  EXPECT_NEAR(solved[0], 1.0, 1e-12);
  EXPECT_NEAR(solved[1], 1.0, 1e-12);
  EXPECT_NEAR(solved[2], 1.0, 1e-12);
  EXPECT_NEAR(
    hl::detail::mahalanobis_squared(residual, covariance, "covariance"),
    14.0,
    1e-12);
}

TEST(Matrix3, ComputesCholeskyAndLogDeterminant)
{
  const hl::detail::Matrix3Storage covariance{
    4.0, 0.0, 0.0,
    0.0, 9.0, 0.0,
    0.0, 0.0, 16.0};
  const auto lower = hl::detail::lower_cholesky(covariance, "covariance");
  EXPECT_NEAR(lower[0], 2.0, 1e-12);
  EXPECT_NEAR(lower[4], 3.0, 1e-12);
  EXPECT_NEAR(lower[8], 4.0, 1e-12);
  EXPECT_NEAR(
    hl::detail::log_determinant_positive_definite(covariance, "covariance"),
    std::log(576.0),
    1e-12);
}

TEST(Matrix3, ReturnsDeterministicDominantEigenvector)
{
  const hl::detail::Matrix3Storage covariance{
    1.0, 0.0, 0.0,
    0.0, 4.0, 0.0,
    0.0, 0.0, 2.0};
  const auto direction = hl::detail::dominant_eigenvector(
    covariance, "covariance");
  EXPECT_NEAR(direction[0], 0.0, 1e-12);
  EXPECT_NEAR(direction[1], 1.0, 1e-12);
  EXPECT_NEAR(direction[2], 0.0, 1e-12);
}

TEST(Matrix3, PositiveSemidefiniteSquareRootPreservesZeroCovariance)
{
  const hl::detail::Matrix3Storage covariance{};
  const auto factor = hl::detail::positive_semidefinite_square_root(
    covariance, 1e-12, "zero covariance");

  for (const double value : factor) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}

TEST(Matrix3, PositiveSemidefiniteSquareRootReconstructsCovariance)
{
  const hl::detail::Matrix3Storage covariance{
    4.0, 0.0, 0.0,
    0.0, 9.0, 0.0,
    0.0, 0.0, 0.0};

  const auto factor_storage = hl::detail::positive_semidefinite_square_root(
    covariance, 1e-12, "semidefinite covariance");
  const auto factor = hl::detail::matrix3_from_row_major(factor_storage);
  const auto reconstructed = factor * factor.transpose();

  EXPECT_NEAR(reconstructed(0, 0), 4.0, 1e-12);
  EXPECT_NEAR(reconstructed(1, 1), 9.0, 1e-12);
  EXPECT_NEAR(reconstructed(2, 2), 0.0, 1e-12);
  EXPECT_NEAR(reconstructed(0, 1), 0.0, 1e-12);
}
