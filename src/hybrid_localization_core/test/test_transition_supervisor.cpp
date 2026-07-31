#include <gtest/gtest.h>

#include <limits>

#include "hybrid_localization_core/transition_supervisor.hpp"

namespace hl = hybrid_localization;

namespace
{

hl::TransitionSupervisorConfig fast_config()
{
  hl::TransitionSupervisorConfig config;
  config.particle_convergence_updates = 2U;
  config.shadow_agreement_updates = 2U;
  config.shadow_failure_updates = 2U;
  config.tracking_bad_updates = 2U;
  config.tracking_ambiguous_updates = 2U;
  config.tracking_recovery_updates = 2U;
  config.local_recovery_success_updates = 2U;
  config.local_recovery_failure_updates = 2U;
  config.global_recovery_success_updates = 2U;
  config.global_recovery_failure_updates = 2U;
  return config;
}

hl::TransitionEvidence evidence(double time)
{
  hl::TransitionEvidence value;
  value.timestamp_seconds = time;
  return value;
}

}  // namespace

TEST(TransitionSupervisor, AdvancesThroughParticleShadowAndMixtureTracking)
{
  hl::TransitionSupervisor supervisor(fast_config());

  auto input = evidence(1.0);
  input.particle_belief_converged = true;
  EXPECT_FALSE(supervisor.update(input).transitioned);
  input.timestamp_seconds = 2.0;
  EXPECT_EQ(supervisor.update(input).state, hl::LocalizationState::particle_converging);

  input.timestamp_seconds = 3.0;
  input.gmm_available = true;
  EXPECT_FALSE(supervisor.update(input).transitioned);
  input.timestamp_seconds = 4.0;
  EXPECT_EQ(supervisor.update(input).state, hl::LocalizationState::mixture_shadow);
  EXPECT_EQ(supervisor.status().authority, hl::LocalizationAuthority::particles);

  input.timestamp_seconds = 5.0;
  input.gmm_health_good = true;
  input.shadow_agreement_good = true;
  EXPECT_FALSE(supervisor.update(input).transitioned);
  input.timestamp_seconds = 6.0;
  const auto status = supervisor.update(input);
  EXPECT_EQ(status.state, hl::LocalizationState::mixture_tracking);
  EXPECT_EQ(status.authority, hl::LocalizationAuthority::gaussian_mixture);
  EXPECT_EQ(status.reason, hl::TransitionReason::mixture_authority_acquired);
}

TEST(TransitionSupervisor, UsesSeparateAmbiguityAndFailureTransitions)
{
  hl::TransitionSupervisor supervisor(
    fast_config(), hl::LocalizationState::mixture_tracking);

  auto input = evidence(1.0);
  input.tracking_ambiguous = true;
  supervisor.update(input);
  input.timestamp_seconds = 2.0;
  EXPECT_EQ(supervisor.update(input).state, hl::LocalizationState::tracking_ambiguous);
  EXPECT_EQ(supervisor.status().authority, hl::LocalizationAuthority::gaussian_mixture);

  input.timestamp_seconds = 3.0;
  input.tracking_ambiguous = false;
  input.gmm_health_good = true;
  supervisor.update(input);
  input.timestamp_seconds = 4.0;
  EXPECT_EQ(supervisor.update(input).state, hl::LocalizationState::mixture_tracking);

  input.timestamp_seconds = 5.0;
  input.gmm_health_good = false;
  input.gmm_health_bad = true;
  supervisor.update(input);
  input.timestamp_seconds = 6.0;
  EXPECT_EQ(supervisor.update(input).state, hl::LocalizationState::local_recovery);
  EXPECT_EQ(supervisor.status().authority, hl::LocalizationAuthority::particles);
}

TEST(TransitionSupervisor, EscalatesAndRecoversThroughRecoveryStates)
{
  hl::TransitionSupervisor supervisor(
    fast_config(), hl::LocalizationState::local_recovery);

  auto input = evidence(1.0);
  input.local_recovery_failed = true;
  supervisor.update(input);
  input.timestamp_seconds = 2.0;
  EXPECT_EQ(supervisor.update(input).state, hl::LocalizationState::global_recovery);

  input.timestamp_seconds = 3.0;
  input.local_recovery_failed = false;
  input.global_recovery_succeeded = true;
  supervisor.update(input);
  input.timestamp_seconds = 4.0;
  EXPECT_EQ(supervisor.update(input).state, hl::LocalizationState::particle_converging);
}

TEST(TransitionSupervisor, DeclaresLostAfterRepeatedGlobalFailure)
{
  hl::TransitionSupervisor supervisor(
    fast_config(), hl::LocalizationState::global_recovery);

  auto input = evidence(1.0);
  input.global_recovery_failed = true;
  supervisor.update(input);
  input.timestamp_seconds = 2.0;
  const auto status = supervisor.update(input);
  EXPECT_EQ(status.state, hl::LocalizationState::localization_lost);
  EXPECT_EQ(status.authority, hl::LocalizationAuthority::none);
  EXPECT_EQ(status.reason, hl::TransitionReason::localization_declared_lost);
}

TEST(TransitionSupervisor, EnforcesDwellAndCooldown)
{
  auto config = fast_config();
  config.particle_convergence_updates = 1U;
  config.minimum_state_dwell_seconds = 5.0;
  config.transition_cooldown_seconds = 3.0;
  hl::TransitionSupervisor supervisor(config);

  auto input = evidence(1.0);
  input.particle_belief_converged = true;
  EXPECT_FALSE(supervisor.update(input).transitioned);

  input.timestamp_seconds = 5.0;
  EXPECT_TRUE(supervisor.update(input).transitioned);
  EXPECT_EQ(supervisor.status().state, hl::LocalizationState::particle_converging);

  input.gmm_available = true;
  input.timestamp_seconds = 6.0;
  supervisor.update(input);
  input.timestamp_seconds = 7.0;
  EXPECT_FALSE(supervisor.update(input).transitioned);
  EXPECT_EQ(supervisor.status().state, hl::LocalizationState::particle_converging);

  input.timestamp_seconds = 10.0;
  EXPECT_TRUE(supervisor.update(input).transitioned);
  EXPECT_EQ(supervisor.status().state, hl::LocalizationState::mixture_shadow);
}

TEST(TransitionSupervisor, EmergencyRecoveryBypassesTemporalGuards)
{
  auto config = fast_config();
  config.minimum_state_dwell_seconds = 100.0;
  config.transition_cooldown_seconds = 100.0;
  hl::TransitionSupervisor supervisor(
    config, hl::LocalizationState::mixture_tracking, 10.0);

  auto input = evidence(10.1);
  input.emergency_global_recovery = true;
  const auto status = supervisor.update(input);
  EXPECT_TRUE(status.transitioned);
  EXPECT_EQ(status.state, hl::LocalizationState::global_recovery);
  EXPECT_EQ(status.authority, hl::LocalizationAuthority::particles);
  EXPECT_EQ(status.reason, hl::TransitionReason::emergency_global_recovery);
}

TEST(TransitionSupervisor, RejectsInvalidConfigurationAndTimestamps)
{
  auto config = fast_config();
  config.shadow_agreement_updates = 0U;
  EXPECT_THROW(
    (void)hl::TransitionSupervisor{config},
    std::invalid_argument);

  config = fast_config();
  config.minimum_state_dwell_seconds = -1.0;
  EXPECT_THROW(
    (void)hl::TransitionSupervisor{config},
    std::invalid_argument);

  config = fast_config();
  hl::TransitionSupervisor supervisor(config);

  auto input = evidence(2.0);
  supervisor.update(input);

  input.timestamp_seconds = 1.0;
  EXPECT_THROW(
    supervisor.update(input),
    std::invalid_argument);

  input.timestamp_seconds =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    supervisor.update(input),
    std::invalid_argument);
}
