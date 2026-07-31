#pragma once

#include <array>
#include <limits>
#include <vector>

#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// A direct SE(2) pose observation.
///
/// Covariance is stored row-major for [x, y, yaw].
struct GaussianPoseObservation
{
  Pose2d mean{};
  std::array<double, 9> covariance{};
};

/// Numerical and statistical controls for Gaussian measurement correction.
struct GaussianUpdateConfig
{
  /// Maximum squared Mahalanobis innovation accepted by a component.
  /// Positive infinity disables gating.
  double maximum_mahalanobis_distance_squared{
    std::numeric_limits<double>::infinity()};

  /// Lower bound used for component likelihood reweighting.
  double likelihood_floor{1e-12};

  /// Diagonal regularization added to the innovation covariance before solving.
  double innovation_covariance_regularization{1e-12};

  /// Numerical covariance validation tolerances.
  double covariance_symmetry_tolerance{1e-12};
  double covariance_psd_tolerance{1e-12};
};

/// Diagnostic result for one component update.
struct GaussianComponentUpdate
{
  GaussianComponent component{};
  Pose2d innovation{};
  double mahalanobis_distance_squared{0.0};
  double likelihood{0.0};
  bool accepted{false};
};

/// Diagnostic result for a complete Gaussian-mixture update.
struct GaussianMixtureUpdate
{
  GaussianMixture mixture{};
  std::vector<GaussianComponentUpdate> component_updates{};
  double normalization_evidence{0.0};
};

/// Correct one Gaussian component from a direct SE(2) pose observation.
///
/// The observation matrix is identity. Yaw innovation is wrapped to [-pi, pi).
/// A rejected component keeps its prior mean and covariance but still receives
/// the configured likelihood floor for later mixture reweighting.
[[nodiscard]] GaussianComponentUpdate update_gaussian_component(
  const GaussianComponent & component,
  const GaussianPoseObservation & observation,
  const GaussianUpdateConfig & config = {});

/// Correct and likelihood-reweight every retained Gaussian component.
///
/// Updated component weights are normalized over the represented mass, so
/// their sum remains 1 - discarded_weight. discarded_weight itself is
/// preserved because this observation model does not spatially represent it.
[[nodiscard]] GaussianMixtureUpdate update_gaussian_mixture(
  const GaussianMixture & mixture,
  const GaussianPoseObservation & observation,
  const GaussianUpdateConfig & config = {});

}  // namespace hybrid_localization