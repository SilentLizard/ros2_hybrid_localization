# Observation-mode summary derived from one weighted particle cloud.
# AMCL remains authoritative while this message is used for diagnostics.

std_msgs/Header header
uint64 analysis_sequence

uint64 particle_count
uint64 retained_cluster_count
float64 effective_sample_size
float64 retained_cluster_weight
float64 noise_weight
float64 dominant_cluster_weight

hybrid_localization_msgs/GaussianMixture mixture
hybrid_localization_msgs/LocalizationHealth health
hybrid_localization_msgs/TransitionEvidence evidence
