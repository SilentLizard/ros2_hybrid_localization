# Instantaneous thresholded evidence supplied to the transition supervisor.
# Temporal hysteresis remains inside the supervisor.

std_msgs/Header header

bool particle_belief_converged
bool gmm_available
bool gmm_health_good
bool gmm_health_bad
bool tracking_ambiguous
bool shadow_agreement_good

bool local_recovery_succeeded
bool local_recovery_failed
bool global_recovery_succeeded
bool global_recovery_failed
bool emergency_global_recovery
