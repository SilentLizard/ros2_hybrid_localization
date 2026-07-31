#pragma once

#include <cstddef>

namespace hybrid_localization
{

enum class LocalizationState
{
  particle_global,
  particle_converging,
  mixture_shadow,
  mixture_tracking,
  tracking_ambiguous,
  local_recovery,
  global_recovery,
  localization_lost
};

enum class LocalizationAuthority
{
  none,
  particles,
  gaussian_mixture
};

enum class TransitionReason
{
  none,
  particle_convergence_started,
  mixture_shadow_started,
  mixture_authority_acquired,
  shadow_validation_failed,
  tracking_became_ambiguous,
  tracking_recovered,
  local_recovery_started,
  global_recovery_started,
  local_recovery_succeeded,
  global_recovery_succeeded,
  localization_declared_lost,
  emergency_global_recovery
};

/// Thresholded, instantaneous evidence supplied to the transition supervisor.
///
/// Raw metric calculation and numerical thresholds remain outside the state
/// machine. The supervisor only applies temporal policy and authority rules.
struct TransitionEvidence
{
  double timestamp_seconds{0.0};

  bool particle_belief_converged{false};
  bool gmm_available{false};
  bool gmm_health_good{false};
  bool gmm_health_bad{false};
  bool tracking_ambiguous{false};
  bool shadow_agreement_good{false};

  bool local_recovery_succeeded{false};
  bool local_recovery_failed{false};
  bool global_recovery_succeeded{false};
  bool global_recovery_failed{false};

  /// Emergency override. This bypasses dwell time and transition cooldown.
  bool emergency_global_recovery{false};
};

/// Temporal policy for supervised representation transitions.
struct TransitionSupervisorConfig
{
  std::size_t particle_convergence_updates{3U};
  std::size_t shadow_agreement_updates{3U};
  std::size_t shadow_failure_updates{2U};
  std::size_t tracking_bad_updates{3U};
  std::size_t tracking_ambiguous_updates{2U};
  std::size_t tracking_recovery_updates{3U};
  std::size_t local_recovery_success_updates{2U};
  std::size_t local_recovery_failure_updates{3U};
  std::size_t global_recovery_success_updates{2U};
  std::size_t global_recovery_failure_updates{5U};

  double minimum_state_dwell_seconds{0.0};
  double transition_cooldown_seconds{0.0};
};

struct TransitionSupervisorStatus
{
  LocalizationState state{LocalizationState::particle_global};
  LocalizationAuthority authority{LocalizationAuthority::particles};

  bool transitioned{false};
  TransitionReason reason{TransitionReason::none};

  double state_entered_at_seconds{0.0};
  double last_transition_at_seconds{0.0};

  std::size_t positive_evidence_count{0U};
  std::size_t negative_evidence_count{0U};
};

/// Stateful ROS-independent transition supervisor.
///
/// Only this object decides which belief representation is authoritative. It
/// does not publish TF or interact with ROS; an integration layer consumes the
/// returned authority and performs any handover continuity checks.
class TransitionSupervisor
{
public:
  explicit TransitionSupervisor(
    const TransitionSupervisorConfig & config = {},
    LocalizationState initial_state = LocalizationState::particle_global,
    double initial_timestamp_seconds = 0.0);

  [[nodiscard]] TransitionSupervisorStatus update(
    const TransitionEvidence & evidence);

  [[nodiscard]] const TransitionSupervisorStatus & status() const noexcept;
  [[nodiscard]] const TransitionSupervisorConfig & config() const noexcept;

  void reset(
    LocalizationState state = LocalizationState::particle_global,
    double timestamp_seconds = 0.0);

private:
  TransitionSupervisorConfig config_{};
  TransitionSupervisorStatus status_{};
  bool has_update_{false};
  double last_update_timestamp_seconds_{0.0};

  [[nodiscard]] bool transition_allowed(
    double timestamp_seconds,
    bool emergency) const noexcept;

  void transition_to(
    LocalizationState state,
    TransitionReason reason,
    double timestamp_seconds);

  void clear_evidence_counts() noexcept;
};

[[nodiscard]] LocalizationAuthority authority_for_state(
  LocalizationState state) noexcept;

}  // namespace hybrid_localization
