#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

namespace hybrid_localization::detail
{

using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;
using Matrix3Storage = std::array<double, 9>;
using Vector3Storage = std::array<double, 3>;

[[nodiscard]] inline Matrix3 matrix3_from_row_major(const Matrix3Storage & values)
{
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mapped(values.data());
  return Matrix3(mapped);
}

[[nodiscard]] inline Matrix3Storage matrix3_to_row_major(const Matrix3 & matrix)
{
  Matrix3Storage values{};
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mapped(values.data());
  mapped = matrix;
  return values;
}

[[nodiscard]] inline Vector3 vector3_from_storage(const Vector3Storage & values)
{
  return Vector3{values[0], values[1], values[2]};
}

[[nodiscard]] inline Vector3Storage vector3_to_storage(const Vector3 & vector)
{
  return Vector3Storage{vector.x(), vector.y(), vector.z()};
}

[[nodiscard]] inline bool is_finite(const Matrix3 & matrix) noexcept
{
  return matrix.array().isFinite().all();
}

[[nodiscard]] inline Matrix3 symmetrized(const Matrix3 & matrix) noexcept
{
  return 0.5 * (matrix + matrix.transpose());
}

inline void validate_covariance(
  const Matrix3Storage & covariance,
  const double symmetry_tolerance,
  const double psd_tolerance,
  const std::string_view name)
{
  if (!std::isfinite(symmetry_tolerance) || symmetry_tolerance < 0.0 ||
    !std::isfinite(psd_tolerance) || psd_tolerance < 0.0)
  {
    throw std::invalid_argument("Covariance tolerances must be finite and nonnegative");
  }

  const Matrix3 matrix = matrix3_from_row_major(covariance);
  const std::string label{name};
  if (!is_finite(matrix)) {
    throw std::invalid_argument(label + " must contain only finite values");
  }
  if ((matrix - matrix.transpose()).cwiseAbs().maxCoeff() > symmetry_tolerance) {
    throw std::invalid_argument(label + " must be symmetric");
  }

  Eigen::SelfAdjointEigenSolver<Matrix3> solver(symmetrized(matrix));
  if (solver.info() != Eigen::Success || !solver.eigenvalues().array().isFinite().all()) {
    throw std::invalid_argument(label + " eigendecomposition failed");
  }
  if (solver.eigenvalues().minCoeff() < -psd_tolerance) {
    throw std::invalid_argument(label + " must be positive semidefinite");
  }
}

[[nodiscard]] inline Matrix3Storage transpose(const Matrix3Storage & matrix)
{
  return matrix3_to_row_major(matrix3_from_row_major(matrix).transpose());
}

[[nodiscard]] inline Matrix3Storage multiply(
  const Matrix3Storage & lhs,
  const Matrix3Storage & rhs)
{
  return matrix3_to_row_major(matrix3_from_row_major(lhs) * matrix3_from_row_major(rhs));
}

[[nodiscard]] inline Vector3Storage multiply(
  const Matrix3Storage & matrix,
  const Vector3Storage & vector)
{
  return vector3_to_storage(matrix3_from_row_major(matrix) * vector3_from_storage(vector));
}

[[nodiscard]] inline Matrix3Storage add(
  const Matrix3Storage & lhs,
  const Matrix3Storage & rhs)
{
  return matrix3_to_row_major(matrix3_from_row_major(lhs) + matrix3_from_row_major(rhs));
}

[[nodiscard]] inline Matrix3Storage subtract(
  const Matrix3Storage & lhs,
  const Matrix3Storage & rhs)
{
  return matrix3_to_row_major(matrix3_from_row_major(lhs) - matrix3_from_row_major(rhs));
}

[[nodiscard]] inline Matrix3Storage identity()
{
  return matrix3_to_row_major(Matrix3::Identity());
}

inline void symmetrize(Matrix3Storage & covariance)
{
  covariance = matrix3_to_row_major(symmetrized(matrix3_from_row_major(covariance)));
}

[[nodiscard]] inline double determinant(const Matrix3Storage & matrix)
{
  return matrix3_from_row_major(matrix).determinant();
}

[[nodiscard]] inline Eigen::LDLT<Matrix3> positive_definite_ldlt(
  const Matrix3 & matrix,
  const std::string_view name)
{
  const std::string label{name};
  if (!is_finite(matrix)) {
    throw std::invalid_argument(label + " must be finite");
  }
  Eigen::LDLT<Matrix3> decomposition(symmetrized(matrix));
  if (decomposition.info() != Eigen::Success || !decomposition.isPositive() ||
    !decomposition.vectorD().array().isFinite().all() ||
    decomposition.vectorD().minCoeff() <= 0.0)
  {
    throw std::domain_error(label + " must be positive definite");
  }
  return decomposition;
}

[[nodiscard]] inline Matrix3Storage inverse_positive_definite(
  const Matrix3Storage & matrix,
  const std::string_view name)
{
  const auto decomposition = positive_definite_ldlt(matrix3_from_row_major(matrix), name);
  const Matrix3 inverse = decomposition.solve(Matrix3::Identity());
  if (decomposition.info() != Eigen::Success || !is_finite(inverse)) {
    throw std::domain_error(std::string(name) + " solve failed");
  }
  return matrix3_to_row_major(inverse);
}

[[nodiscard]] inline Vector3Storage solve_positive_definite(
  const Matrix3Storage & matrix,
  const Vector3Storage & rhs,
  const std::string_view name)
{
  const auto decomposition = positive_definite_ldlt(matrix3_from_row_major(matrix), name);
  const Vector3 solution = decomposition.solve(vector3_from_storage(rhs));
  if (decomposition.info() != Eigen::Success || !solution.array().isFinite().all()) {
    throw std::domain_error(std::string(name) + " solve failed");
  }
  return vector3_to_storage(solution);
}

[[nodiscard]] inline double quadratic_form(
  const Vector3Storage & vector,
  const Matrix3Storage & matrix)
{
  const Vector3 eigen_vector = vector3_from_storage(vector);
  return eigen_vector.dot(matrix3_from_row_major(matrix) * eigen_vector);
}

[[nodiscard]] inline double mahalanobis_squared(
  const Vector3Storage & residual,
  const Matrix3Storage & covariance,
  const std::string_view name)
{
  const Vector3 eigen_residual = vector3_from_storage(residual);
  const auto decomposition = positive_definite_ldlt(matrix3_from_row_major(covariance), name);
  const Vector3 solution = decomposition.solve(eigen_residual);
  const double value = eigen_residual.dot(solution);
  if (!std::isfinite(value)) {
    throw std::domain_error(std::string(name) + " produced a non-finite Mahalanobis distance");
  }
  return std::max(0.0, value);
}

[[nodiscard]] inline double log_determinant_positive_definite(
  const Matrix3Storage & matrix,
  const std::string_view name)
{
  const auto decomposition = positive_definite_ldlt(matrix3_from_row_major(matrix), name);
  return decomposition.vectorD().array().log().sum();
}

[[nodiscard]] inline Matrix3Storage lower_cholesky(
  const Matrix3Storage & covariance,
  const std::string_view name)
{
  Eigen::LLT<Matrix3> decomposition(symmetrized(matrix3_from_row_major(covariance)));
  if (decomposition.info() != Eigen::Success) {
    throw std::domain_error(std::string(name) + " must be positive definite");
  }
  return matrix3_to_row_major(Matrix3(decomposition.matrixL()));
}

[[nodiscard]] inline Matrix3Storage positive_semidefinite_square_root(
  const Matrix3Storage & covariance,
  const double psd_tolerance,
  const std::string_view name)
{
  if (!std::isfinite(psd_tolerance) || psd_tolerance < 0.0) {
    throw std::invalid_argument("PSD tolerance must be finite and nonnegative");
  }

  const Matrix3 matrix = symmetrized(matrix3_from_row_major(covariance));
  const std::string label{name};
  if (!is_finite(matrix)) {
    throw std::invalid_argument(label + " must contain only finite values");
  }

  Eigen::SelfAdjointEigenSolver<Matrix3> solver(matrix);
  if (solver.info() != Eigen::Success ||
    !solver.eigenvalues().array().isFinite().all())
  {
    throw std::domain_error(label + " eigendecomposition failed");
  }

  if (solver.eigenvalues().minCoeff() < -psd_tolerance) {
    throw std::invalid_argument(label + " must be positive semidefinite");
  }

  Vector3 square_roots{};
  for (Eigen::Index index = 0; index < square_roots.size(); ++index) {
    square_roots[index] = std::sqrt(std::max(0.0, solver.eigenvalues()[index]));
  }

  const Matrix3 factor =
    solver.eigenvectors() * square_roots.asDiagonal();

  if (!is_finite(factor)) {
    throw std::domain_error(label + " square root is invalid");
  }

  return matrix3_to_row_major(factor);
}

[[nodiscard]] inline Vector3Storage dominant_eigenvector(
  const Matrix3Storage & covariance,
  const std::string_view name)
{
  const Matrix3 matrix = symmetrized(matrix3_from_row_major(covariance));
  Eigen::SelfAdjointEigenSolver<Matrix3> solver(matrix);
  if (solver.info() != Eigen::Success) {
    throw std::domain_error(std::string(name) + " eigendecomposition failed");
  }
  Vector3 direction = solver.eigenvectors().col(2);
  if (!direction.array().isFinite().all()) {
    throw std::domain_error(std::string(name) + " dominant eigenvector is invalid");
  }
  for (Eigen::Index index = 0; index < direction.size(); ++index) {
    if (std::abs(direction[index]) > 1e-15) {
      if (direction[index] < 0.0) {
        direction = -direction;
      }
      break;
    }
  }
  return vector3_to_storage(direction);
}

}  // namespace hybrid_localization::detail
