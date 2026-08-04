#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "hybrid_localization_core/gaussian_mixture_management.hpp"
#include "hybrid_localization_core/gaussian_mixture_merging.hpp"
#include "hybrid_localization_core/gaussian_mixture_prediction.hpp"
#include "hybrid_localization_core/gaussian_mixture_splitting.hpp"
#include "hybrid_localization_core/gaussian_mixture_update.hpp"
#include "hybrid_localization_core/localization_health.hpp"

namespace hybrid_localization
{

/// Fit-quality evidence addressed by stable hypothesis identity rather than
/// transient component index.
struct GaussianTrackerSplitEvidence
{
  HypothesisId hypothesis_id{invalid_hypothesis_id};
  std::span<const std::size_t> source_indices{};
  GaussianFitQuality fit_quality{};
};

/// Complete configuration for one recursive bounded-GMM update cycle.
struct GaussianMixtureTrackerConfig
{
  GaussianPredictionConfig prediction{};
  GaussianUpdateConfig update{};
  GaussianMixtureManagementConfig management{};
  GaussianMixtureMergingConfig merging{};
  GaussianMixtureSplittingConfig splitting{};

  /// Hard upper bound enforced both before and after topology changes.
  std::size_t maximum_component_count{8U};

  /// Enables evidence-driven splitting when particles and keyed evidence are
  /// supplied to update().
  bool enable_splitting{true};
};

/// Inputs for one recursive tracker update.
struct GaussianMixtureTrackerInput
{
  GaussianMotionIncrement motion{};
  std::optional<GaussianPoseObservation> observation{};

  /// Optional source particles and ID-keyed evidence for bounded splitting.
  std::span<const WeightedParticle> source_particles{};
  std::span<const GaussianTrackerSplitEvidence> split_evidence{};
};

/// Stage-by-stage diagnostics for one recursive update.
struct GaussianMixtureTrackerResult
{
  std::size_t update_sequence{0U};
  GaussianMixture mixture{};

  bool measurement_applied{false};
  std::optional<GaussianMixtureUpdate> measurement_update{};

  GaussianMixtureManagementResult pre_topology_management{};
  GaussianMixtureMergingResult merging{};
  std::optional<GaussianMixtureSplittingResult> splitting{};
  GaussianMixtureManagementResult final_management{};

  LocalizationHealthMetrics pre_topology_health{};
  LocalizationHealthMetrics final_health{};

  /// Evidence entries whose hypotheses disappeared through merging before the
  /// split stage. They are reported rather than treated as errors.
  std::vector<HypothesisId> superseded_split_evidence_ids{};

  bool component_limit_enforced{false};
};

/// Stateful ROS-independent recursive Gaussian-mixture tracker.
///
/// Each update executes:
///
///   predict -> optional correct -> normalize/prune -> merge
///           -> optional bounded split -> enforce hard component cap -> health
///
/// One monotonic ID generator is owned for the tracker lifetime and is passed
/// to every operation that creates hypotheses.
class GaussianMixtureTracker
{
public:
  explicit GaussianMixtureTracker(
    const GaussianMixture & initial_mixture,
    const GaussianMixtureTrackerConfig & config = {});

  [[nodiscard]] GaussianMixtureTrackerResult update(
    const GaussianMixtureTrackerInput & input);

  void reset(const GaussianMixture & mixture);

  [[nodiscard]] const GaussianMixture & mixture() const noexcept;
  [[nodiscard]] const GaussianMixtureTrackerConfig & config() const noexcept;
  [[nodiscard]] std::size_t update_sequence() const noexcept;
  [[nodiscard]] HypothesisId next_hypothesis_id() const noexcept;

private:
  GaussianMixtureTrackerConfig config_{};
  GaussianMixture mixture_{};
  HypothesisIdGenerator id_generator_;
  std::size_t update_sequence_{0U};
};

}  // namespace hybrid_localization
