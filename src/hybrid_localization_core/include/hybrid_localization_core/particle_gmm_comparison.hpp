#pragma once

#include <array>
#include <cstddef>
#include <span>

#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

/// Numerical and support controls for particle/GMM shadow comparison.
struct ParticleGmmComparisonConfig
{
  /// Diagonal regularization used when solving covariance systems.
  double covariance_regularization{1e-9};

  /// Squared Mahalanobis gate used to classify particle support by any
  /// retained Gaussian component.
  double particle_support_mahalanobis_distance_squared{9.0};

  double covariance_symmetry_tolerance{1e-12};
  double covariance_psd_tolerance{1e-12};
  double mass_tolerance{1e-12};
};

/// Raw particle/GMM agreement evidence for shadow-mode supervision.
///
/// The result intentionally does not classify the representations as agreeing
/// or disagreeing. Transition thresholds and temporal hysteresis belong to the
/// transition supervisor.
struct ParticleGmmComparison
{
  Pose2d particle_mean{};
  std::array<double, 9> particle_covariance{};

  Pose2d gmm_mean{};
  std::array<double, 9> gmm_covariance{};

  double position_difference{0.0};
  double absolute_yaw_difference{0.0};
  double belief_mahalanobis_distance_squared{0.0};

  std::size_t dominant_component_index{0U};
  double dominant_component_position_difference{0.0};
  double dominant_component_absolute_yaw_difference{0.0};
  double dominant_component_mahalanobis_distance_squared{0.0};

  /// Normalized particle probability mass that falls inside the configured
  /// Mahalanobis gate of at least one retained Gaussian component.
  double particle_mass_supported_by_gmm{0.0};

  double represented_gmm_weight{0.0};
  double discarded_gmm_weight{0.0};
};

/// Compare a normalized particle belief against a retained Gaussian mixture.
///
/// The GMM is moment matched over represented component mass. Particle support
/// is evaluated against every retained component, preserving multimodal
/// evidence that a global mean-only comparison could hide.
[[nodiscard]] ParticleGmmComparison compare_particle_and_gmm_beliefs(
  std::span<const WeightedParticle> particles,
  const GaussianMixture & mixture,
  const ParticleGmmComparisonConfig & config = {});

}  // namespace hybrid_localization
