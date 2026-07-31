#include "hybrid_localization_core/transition_supervisor.hpp"

#include <cmath>
#include <stdexcept>

namespace hybrid_localization
{
namespace
{

void validate_positive_count(std::size_t value, const char * name)
{
  if (value == 0U) {
    throw std::invalid_argument(std::string(name) + " must be greater than zero");
  }
}

void validate_config(const TransitionSupervisorConfig & config)
{
  validate_positive_count(config.particle_convergence_updates, "particle_convergence_updates");
  validate_positive_count(config.shadow_agreement_updates, "shadow_agreement_updates");
  validate_positive_count(config.shadow_failure_updates, "shadow_failure_updates");
  validate_positive_count(config.tracking_bad_updates, "tracking_bad_updates");
  validate_positive_count(config.tracking_ambiguous_updates, "tracking_ambiguous_updates");
  validate_positive_count(config.tracking_recovery_updates, "tracking_recovery_updates");
  validate_positive_count(config.local_recovery_success_updates, "local_recovery_success_updates");
  validate_positive_count(config.local_recovery_failure_updates, "local_recovery_failure_updates");
  validate_positive_count(config.global_recovery_success_updates, "global_recovery_success_updates");
  validate_positive_count(config.global_recovery_failure_updates, "global_recovery_failure_updates");

  if (!std::isfinite(config.minimum_state_dwell_seconds) ||
    config.minimum_state_dwell_seconds < 0.0)
  {
    throw std::invalid_argument("minimum_state_dwell_seconds must be finite and nonnegative");
  }
  if (!std::isfinite(config.transition_cooldown_seconds) ||
    config.transition_cooldown_seconds < 0.0)
  {
    throw std::invalid_argument("transition_cooldown_seconds must be finite and nonnegative");
  }
}

void validate_timestamp(double timestamp_seconds)
{
  if (!std::isfinite(timestamp_seconds) || timestamp_seconds < 0.0) {
    throw std::invalid_argument("timestamp_seconds must be finite and nonnegative");
  }
}

void increment_or_clear(bool condition, std::size_t & counter)
{
  if (condition) {
    ++counter;
  } else {
    counter = 0U;
  }
}

}  // namespace

LocalizationAuthority authority_for_state(LocalizationState state) noexcept
{
  switch (state) {
    case LocalizationState::mixture_tracking:
    case LocalizationState::tracking_ambiguous:
      return LocalizationAuthority::gaussian_mixture;
    case LocalizationState::localization_lost:
      return LocalizationAuthority::none;
    case LocalizationState::particle_global:
    case LocalizationState::particle_converging:
    case LocalizationState::mixture_shadow:
    case LocalizationState::local_recovery:
    case LocalizationState::global_recovery:
      return LocalizationAuthority::particles;
  }
  return LocalizationAuthority::none;
}

TransitionSupervisor::TransitionSupervisor(
  const TransitionSupervisorConfig & config,
  LocalizationState initial_state,
  double initial_timestamp_seconds)
: config_(config)
{
  validate_config(config_);
  reset(initial_state, initial_timestamp_seconds);
}

const TransitionSupervisorStatus & TransitionSupervisor::status() const noexcept
{
  return status_;
}

const TransitionSupervisorConfig & TransitionSupervisor::config() const noexcept
{
  return config_;
}

void TransitionSupervisor::reset(
  LocalizationState state,
  double timestamp_seconds)
{
  validate_timestamp(timestamp_seconds);
  status_ = {};
  status_.state = state;
  status_.authority = authority_for_state(state);
  status_.state_entered_at_seconds = timestamp_seconds;
  status_.last_transition_at_seconds = timestamp_seconds;
  has_update_ = false;
  last_update_timestamp_seconds_ = timestamp_seconds;
}

bool TransitionSupervisor::transition_allowed(
  double timestamp_seconds,
  bool emergency) const noexcept
{
  if (emergency) {
    return true;
  }
  const double state_age = timestamp_seconds - status_.state_entered_at_seconds;
  const double cooldown_age = timestamp_seconds - status_.last_transition_at_seconds;
  return state_age >= config_.minimum_state_dwell_seconds &&
         cooldown_age >= config_.transition_cooldown_seconds;
}

void TransitionSupervisor::clear_evidence_counts() noexcept
{
  status_.positive_evidence_count = 0U;
  status_.negative_evidence_count = 0U;
}

void TransitionSupervisor::transition_to(
  LocalizationState state,
  TransitionReason reason,
  double timestamp_seconds)
{
  status_.state = state;
  status_.authority = authority_for_state(state);
  status_.transitioned = true;
  status_.reason = reason;
  status_.state_entered_at_seconds = timestamp_seconds;
  status_.last_transition_at_seconds = timestamp_seconds;
  clear_evidence_counts();
}

TransitionSupervisorStatus TransitionSupervisor::update(
  const TransitionEvidence & evidence)
{
  validate_timestamp(evidence.timestamp_seconds);
  if (has_update_ && evidence.timestamp_seconds < last_update_timestamp_seconds_) {
    throw std::invalid_argument("transition evidence timestamps must be monotonic");
  }
  has_update_ = true;
  last_update_timestamp_seconds_ = evidence.timestamp_seconds;
  status_.transitioned = false;
  status_.reason = TransitionReason::none;

  if (evidence.emergency_global_recovery &&
    status_.state != LocalizationState::global_recovery)
  {
    transition_to(
      LocalizationState::global_recovery,
      TransitionReason::emergency_global_recovery,
      evidence.timestamp_seconds);
    return status_;
  }

  switch (status_.state) {
    case LocalizationState::particle_global:
      increment_or_clear(
        evidence.particle_belief_converged,
        status_.positive_evidence_count);
      if (status_.positive_evidence_count >= config_.particle_convergence_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::particle_converging,
          TransitionReason::particle_convergence_started,
          evidence.timestamp_seconds);
      }
      break;

    case LocalizationState::particle_converging:
      increment_or_clear(
        evidence.particle_belief_converged && evidence.gmm_available,
        status_.positive_evidence_count);
      increment_or_clear(
        !evidence.particle_belief_converged,
        status_.negative_evidence_count);
      if (status_.negative_evidence_count >= config_.shadow_failure_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::particle_global,
          TransitionReason::shadow_validation_failed,
          evidence.timestamp_seconds);
      } else if (status_.positive_evidence_count >= config_.particle_convergence_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::mixture_shadow,
          TransitionReason::mixture_shadow_started,
          evidence.timestamp_seconds);
      }
      break;

    case LocalizationState::mixture_shadow:
      increment_or_clear(
        evidence.gmm_available && evidence.gmm_health_good &&
        evidence.shadow_agreement_good,
        status_.positive_evidence_count);
      increment_or_clear(
        !evidence.gmm_available || evidence.gmm_health_bad ||
        !evidence.shadow_agreement_good,
        status_.negative_evidence_count);
      if (status_.negative_evidence_count >= config_.shadow_failure_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::particle_converging,
          TransitionReason::shadow_validation_failed,
          evidence.timestamp_seconds);
      } else if (status_.positive_evidence_count >= config_.shadow_agreement_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::mixture_tracking,
          TransitionReason::mixture_authority_acquired,
          evidence.timestamp_seconds);
      }
      break;

    case LocalizationState::mixture_tracking:
      increment_or_clear(evidence.gmm_health_bad, status_.negative_evidence_count);
      increment_or_clear(evidence.tracking_ambiguous, status_.positive_evidence_count);
      if (status_.negative_evidence_count >= config_.tracking_bad_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::local_recovery,
          TransitionReason::local_recovery_started,
          evidence.timestamp_seconds);
      } else if (status_.positive_evidence_count >= config_.tracking_ambiguous_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::tracking_ambiguous,
          TransitionReason::tracking_became_ambiguous,
          evidence.timestamp_seconds);
      }
      break;

    case LocalizationState::tracking_ambiguous:
      increment_or_clear(
        evidence.gmm_health_good && !evidence.tracking_ambiguous,
        status_.positive_evidence_count);
      increment_or_clear(evidence.gmm_health_bad, status_.negative_evidence_count);
      if (status_.negative_evidence_count >= config_.tracking_bad_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::local_recovery,
          TransitionReason::local_recovery_started,
          evidence.timestamp_seconds);
      } else if (status_.positive_evidence_count >= config_.tracking_recovery_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::mixture_tracking,
          TransitionReason::tracking_recovered,
          evidence.timestamp_seconds);
      }
      break;

    case LocalizationState::local_recovery:
      increment_or_clear(
        evidence.local_recovery_succeeded,
        status_.positive_evidence_count);
      increment_or_clear(
        evidence.local_recovery_failed,
        status_.negative_evidence_count);
      if (status_.negative_evidence_count >= config_.local_recovery_failure_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::global_recovery,
          TransitionReason::global_recovery_started,
          evidence.timestamp_seconds);
      } else if (status_.positive_evidence_count >= config_.local_recovery_success_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::mixture_shadow,
          TransitionReason::local_recovery_succeeded,
          evidence.timestamp_seconds);
      }
      break;

    case LocalizationState::global_recovery:
      increment_or_clear(
        evidence.global_recovery_succeeded,
        status_.positive_evidence_count);
      increment_or_clear(
        evidence.global_recovery_failed,
        status_.negative_evidence_count);
      if (status_.negative_evidence_count >= config_.global_recovery_failure_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::localization_lost,
          TransitionReason::localization_declared_lost,
          evidence.timestamp_seconds);
      } else if (status_.positive_evidence_count >= config_.global_recovery_success_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::particle_converging,
          TransitionReason::global_recovery_succeeded,
          evidence.timestamp_seconds);
      }
      break;

    case LocalizationState::localization_lost:
      increment_or_clear(
        evidence.global_recovery_succeeded,
        status_.positive_evidence_count);
      if (status_.positive_evidence_count >= config_.global_recovery_success_updates &&
        transition_allowed(evidence.timestamp_seconds, false))
      {
        transition_to(
          LocalizationState::particle_converging,
          TransitionReason::global_recovery_succeeded,
          evidence.timestamp_seconds);
      }
      break;
  }

  return status_;
}

}  // namespace hybrid_localization
