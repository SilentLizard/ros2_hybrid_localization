#include "hybrid_localization_ros/particle_analysis_processor.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "hybrid_localization_core/gaussian_mixture.hpp"
#include "hybrid_localization_core/localization_health.hpp"
#include "hybrid_localization_core/particle_statistics.hpp"
#include "hybrid_localization_msgs/msg/gaussian_component.hpp"
#include "hybrid_localization_msgs/msg/hypothesis_provenance.hpp"

namespace hybrid_localization_ros
{
namespace
{

[[nodiscard]] double stamp_seconds(const std_msgs::msg::Header & header)
{
  const double seconds = static_cast<double>(header.stamp.sec);
  const double nanoseconds = static_cast<double>(header.stamp.nanosec) * 1e-9;
  const double result = seconds + nanoseconds;

  if (!std::isfinite(result) || result < 0.0) {
    throw std::invalid_argument(
            "Particle-analysis source timestamp must be finite and non-negative");
  }
  return result;
}

[[nodiscard]] geometry_msgs::msg::Pose pose_message(
  const hybrid_localization::Pose2d & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.x;
  message.position.y = pose.y;
  message.position.z = 0.0;

  const double half_yaw = 0.5 * pose.yaw;
  message.orientation.x = 0.0;
  message.orientation.y = 0.0;
  message.orientation.z = std::sin(half_yaw);
  message.orientation.w = std::cos(half_yaw);
  return message;
}

[[nodiscard]] hybrid_localization_msgs::msg::HypothesisProvenance provenance_message(
  const hybrid_localization::HypothesisProvenance & provenance)
{
  hybrid_localization_msgs::msg::HypothesisProvenance message;
  message.id = provenance.id;
  message.parent_ids = provenance.parent_ids;
  message.parent_count = provenance.parent_count;
  message.generation = provenance.generation;
  message.event = static_cast<std::uint8_t>(provenance.event);
  return message;
}

[[nodiscard]] hybrid_localization_msgs::msg::GaussianComponent component_message(
  const hybrid_localization::GaussianComponent & component)
{
  hybrid_localization_msgs::msg::GaussianComponent message;
  message.pose = pose_message(component.mean);
  message.covariance = component.covariance;
  message.weight = component.weight;
  message.sample_count = static_cast<std::uint64_t>(component.sample_count);
  message.provenance = provenance_message(component.provenance);
  return message;
}

[[nodiscard]] hybrid_localization_msgs::msg::GaussianMixture mixture_message(
  const std_msgs::msg::Header & header,
  const std::uint64_t sequence,
  const hybrid_localization::GaussianMixture & mixture)
{
  hybrid_localization_msgs::msg::GaussianMixture message;
  message.header = header;
  message.update_sequence = sequence;
  message.discarded_weight = mixture.discarded_weight;
  message.components.reserve(mixture.components.size());

  for (const auto & component : mixture.components) {
    message.components.push_back(component_message(component));
  }
  return message;
}

[[nodiscard]] hybrid_localization_msgs::msg::LocalizationHealth health_message(
  const std_msgs::msg::Header & header,
  const std::uint64_t sequence,
  const hybrid_localization::LocalizationHealthMetrics & health)
{
  hybrid_localization_msgs::msg::LocalizationHealth message;
  message.header = header;
  message.update_sequence = sequence;
  message.component_count = static_cast<std::uint64_t>(health.component_count);
  message.represented_weight = health.represented_weight;
  message.discarded_weight = health.discarded_weight;
  message.dominant_component_weight = health.dominant_component_weight;
  message.normalized_mixture_entropy = health.normalized_mixture_entropy;
  message.effective_component_count = health.effective_component_count;
  message.weighted_position_variance = health.weighted_position_variance;
  message.weighted_yaw_variance = health.weighted_yaw_variance;
  message.maximum_position_variance = health.maximum_position_variance;
  message.maximum_yaw_variance = health.maximum_yaw_variance;
  message.has_measurement_update = health.has_measurement_update;
  message.accepted_component_fraction = health.accepted_component_fraction;
  message.accepted_component_weight_fraction = health.accepted_component_weight_fraction;
  message.weighted_mahalanobis_distance_squared =
    health.weighted_mahalanobis_distance_squared;
  message.maximum_mahalanobis_distance_squared =
    health.maximum_mahalanobis_distance_squared;
  message.normalization_evidence = health.normalization_evidence;
  message.has_fit_quality = health.has_fit_quality;
  message.weighted_fit_mean_mahalanobis_distance =
    health.weighted_fit_mean_mahalanobis_distance;
  message.maximum_fit_mahalanobis_distance = health.maximum_fit_mahalanobis_distance;
  message.minimum_angular_resultant_length = health.minimum_angular_resultant_length;
  message.has_recovery_failure_score = health.has_recovery_failure_score;
  message.recovery_failure_score = health.recovery_failure_score;
  return message;
}

[[nodiscard]] hybrid_localization_msgs::msg::TransitionEvidence evidence_message(
  const std_msgs::msg::Header & header,
  const hybrid_localization::TransitionEvidence & evidence)
{
  hybrid_localization_msgs::msg::TransitionEvidence message;
  message.header = header;
  message.particle_belief_converged = evidence.particle_belief_converged;
  message.gmm_available = evidence.gmm_available;
  message.gmm_health_good = evidence.gmm_health_good;
  message.gmm_health_bad = evidence.gmm_health_bad;
  message.tracking_ambiguous = evidence.tracking_ambiguous;
  message.shadow_agreement_good = evidence.shadow_agreement_good;
  message.local_recovery_succeeded = evidence.local_recovery_succeeded;
  message.local_recovery_failed = evidence.local_recovery_failed;
  message.global_recovery_succeeded = evidence.global_recovery_succeeded;
  message.global_recovery_failed = evidence.global_recovery_failed;
  message.emergency_global_recovery = evidence.emergency_global_recovery;
  return message;
}

}  // namespace

ParticleAnalysisProcessor::ParticleAnalysisProcessor(
  ParticleAnalysisProcessorConfig config)
: config_(std::move(config))
{
  hybrid_localization::validate_particle_clustering_config(config_.clustering);
  hybrid_localization::validate_localization_evidence_policy_config(
    config_.evidence_policy);
}

ParticleAnalysisProducts ParticleAnalysisProcessor::process(
  const AdaptedParticleCloud & cloud)
{
  if (analysis_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Particle-analysis sequence is exhausted");
  }

  const std::uint64_t next_sequence = analysis_sequence_ + 1U;

  // Work transactionally: hypothesis IDs and sequence are committed only after
  // every core stage and message conversion succeeds.
  auto working_id_generator = hypothesis_id_generator_;

  const double neff = hybrid_localization::effective_sample_size(cloud.particles);
  const auto clustering = hybrid_localization::cluster_particles(
    cloud.particles,
    config_.clustering);
  const auto particle_metrics = hybrid_localization::summarize_particle_belief(
    cloud.particles,
    clustering);
  const auto mixture = hybrid_localization::fit_gaussian_mixture(
    cloud.particles,
    clustering,
    working_id_generator);

  // Issue #29 evaluates representation-level health only. Measurement-update,
  // Gaussian-fit, recovery, and shadow-comparison evidence are added by later
  // runtime stages, so their corresponding health fields remain explicitly
  // unavailable rather than being synthesized here.
  const auto health = hybrid_localization::evaluate_localization_health(mixture);

  hybrid_localization::LocalizationEvidencePolicyInput policy_input;
  policy_input.timestamp_seconds = stamp_seconds(cloud.header);
  policy_input.state = config_.evidence_state;
  policy_input.particle_metrics = particle_metrics;
  policy_input.gmm_health = health;

  const auto evidence = hybrid_localization::build_transition_evidence(
    policy_input,
    config_.evidence_policy);

  ParticleAnalysisProducts products;
  products.mixture = mixture_message(cloud.header, next_sequence, mixture);
  products.health = health_message(cloud.header, next_sequence, health);
  products.evidence = evidence_message(cloud.header, evidence);

  products.analysis.header = cloud.header;
  products.analysis.analysis_sequence = next_sequence;
  products.analysis.particle_count =
    static_cast<std::uint64_t>(particle_metrics.particle_count);
  products.analysis.retained_cluster_count =
    static_cast<std::uint64_t>(particle_metrics.retained_cluster_count);
  products.analysis.effective_sample_size = neff;
  products.analysis.retained_cluster_weight = particle_metrics.retained_cluster_weight;
  products.analysis.noise_weight = particle_metrics.noise_weight;
  products.analysis.dominant_cluster_weight = particle_metrics.dominant_cluster_weight;
  products.analysis.mixture = products.mixture;
  products.analysis.health = products.health;
  products.analysis.evidence = products.evidence;

  analysis_sequence_ = next_sequence;
  hypothesis_id_generator_ = working_id_generator;
  return products;
}

void ParticleAnalysisProcessor::set_config(ParticleAnalysisProcessorConfig config)
{
  hybrid_localization::validate_particle_clustering_config(config.clustering);
  hybrid_localization::validate_localization_evidence_policy_config(
    config.evidence_policy);

  config_ = std::move(config);
}

std::uint64_t ParticleAnalysisProcessor::analysis_sequence() const noexcept
{
  return analysis_sequence_;
}

const ParticleAnalysisProcessorConfig & ParticleAnalysisProcessor::config() const noexcept
{
  return config_;
}

}  // namespace hybrid_localization_ros
