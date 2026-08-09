#pragma once

#include <cstdint>

#include "hybrid_localization_core/hypothesis_provenance.hpp"
#include "hybrid_localization_core/localization_evidence_policy.hpp"
#include "hybrid_localization_core/particle_clustering.hpp"
#include "hybrid_localization_msgs/msg/gaussian_mixture.hpp"
#include "hybrid_localization_msgs/msg/localization_health.hpp"
#include "hybrid_localization_msgs/msg/particle_analysis.hpp"
#include "hybrid_localization_msgs/msg/transition_evidence.hpp"
#include "hybrid_localization_ros/amcl_particle_cloud_adapter.hpp"

namespace hybrid_localization_ros
{

/// Configuration for one stateless observation-mode particle analysis.
///
/// Runtime parameter exposure is intentionally deferred to issue #11. Issue #29
/// uses the validated core defaults so the ROS node can exercise the complete
/// particle -> clustering -> GMM -> health -> evidence path first.
struct ParticleAnalysisProcessorConfig
{
  hybrid_localization::ParticleClusteringConfig clustering{};
  hybrid_localization::LocalizationEvidencePolicyConfig evidence_policy{};

  /// Observation mode does not run the transition supervisor. This state is
  /// used only for state-aware numerical ambiguity thresholds in the evidence
  /// policy. particle_converging matches AMCL-authoritative Phase B1 behavior.
  hybrid_localization::LocalizationState evidence_state{
    hybrid_localization::LocalizationState::particle_converging};
};

/// The aggregate message plus the individual products published for inspection.
struct ParticleAnalysisProducts
{
  hybrid_localization_msgs::msg::ParticleAnalysis analysis{};
  hybrid_localization_msgs::msg::GaussianMixture mixture{};
  hybrid_localization_msgs::msg::LocalizationHealth health{};
  hybrid_localization_msgs::msg::TransitionEvidence evidence{};
};

/// Convert one adapted AMCL cloud into the complete observation-mode analysis.
///
/// The processor owns only analysis sequencing and a monotonic hypothesis-ID
/// allocator. It does not own localization authority, temporal transition
/// hysteresis, TF, lifecycle state, or GMM recursion.
class ParticleAnalysisProcessor
{
public:
  explicit ParticleAnalysisProcessor(
    ParticleAnalysisProcessorConfig config = {});

  [[nodiscard]] ParticleAnalysisProducts process(
    const AdaptedParticleCloud & cloud);
  /// Replace the validated runtime configuration without resetting analysis
  /// sequence state or hypothesis-ID allocation.
  void set_config(ParticleAnalysisProcessorConfig config);

  [[nodiscard]] std::uint64_t analysis_sequence() const noexcept;
  [[nodiscard]] const ParticleAnalysisProcessorConfig & config() const noexcept;

private:
  ParticleAnalysisProcessorConfig config_{};
  std::uint64_t analysis_sequence_{0U};
  hybrid_localization::HypothesisIdGenerator hypothesis_id_generator_{};
};

}  // namespace hybrid_localization_ros
