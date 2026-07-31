#pragma once

#include <array>

#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// A relative SE(2) motion expressed in the component's local/body frame.
///
/// Covariance is stored row-major for [delta_x, delta_y, delta_yaw].
struct GaussianMotionIncrement
{
  Pose2d mean{};
  std::array<double, 9> covariance{};
};

/// Numerical safeguards applied after covariance propagation.
struct GaussianPredictionConfig
{
  double minimum_position_variance{0.0};
  double minimum_yaw_variance{0.0};
  double covariance_symmetry_tolerance{1e-12};
  double covariance_psd_tolerance{1e-12};
};

/// Predict one Gaussian component through a body-frame SE(2) increment.
///
/// The mean is composed with the relative motion. Covariance is propagated as
///
///   P' = F P F^T + G Q G^T
///
/// where F is the state Jacobian and G is the motion Jacobian.
[[nodiscard]] GaussianComponent predict_gaussian_component(
  const GaussianComponent & component,
  const GaussianMotionIncrement & motion,
  const GaussianPredictionConfig & config = {});

/// Predict every retained component while preserving mixture probability mass.
[[nodiscard]] GaussianMixture predict_gaussian_mixture(
  const GaussianMixture & mixture,
  const GaussianMotionIncrement & motion,
  const GaussianPredictionConfig & config = {});

}  // namespace hybrid_localization