#include "hybrid_localization_core/gaussian_mixture_tracker.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hybrid_localization
{
namespace
{

[[nodiscard]] HypothesisId first_available_id(const GaussianMixture & mixture)
{
  HypothesisId largest_id = invalid_hypothesis_id;
  for (const auto & component : mixture.components) {
    if (has_hypothesis_id(component.provenance)) {
      largest_id = std::max(largest_id, component.provenance.id);
    }
  }

  if (largest_id == std::numeric_limits<HypothesisId>::max()) {
    throw std::overflow_error("No hypothesis IDs remain for tracker allocation");
  }
  return largest_id + 1U;
}

void validate_tracker_config(const GaussianMixtureTrackerConfig & config)
{
  if (config.maximum_component_count == 0U) {
    throw std::invalid_argument("maximum_component_count must be greater than zero");
  }
}

void validate_hypothesis_ids(const GaussianMixture & mixture)
{
  std::unordered_set<HypothesisId> ids;
  ids.reserve(mixture.components.size());

  for (const auto & component : mixture.components) {
    validate_hypothesis_provenance(component.provenance, "Tracker component provenance");
    if (!has_hypothesis_id(component.provenance)) {
      throw std::invalid_argument("Every tracker component requires a hypothesis ID");
    }
    if (!ids.insert(component.provenance.id).second) {
      throw std::invalid_argument("Tracker mixture contains duplicate hypothesis IDs");
    }
  }
}

[[nodiscard]] GaussianMixtureManagementConfig bounded_management_config(
  const GaussianMixtureTrackerConfig & config)
{
  auto management = config.management;
  management.maximum_component_count = config.maximum_component_count;
  return management;
}

[[nodiscard]] std::unordered_map<HypothesisId, const GaussianTrackerSplitEvidence *>
index_split_evidence(
  const std::span<const GaussianTrackerSplitEvidence> evidence,
  const GaussianMixture & pre_merge_mixture)
{
  std::unordered_set<HypothesisId> known_ids;
  known_ids.reserve(pre_merge_mixture.components.size());
  for (const auto & component : pre_merge_mixture.components) {
    known_ids.insert(component.provenance.id);
  }

  std::unordered_map<HypothesisId, const GaussianTrackerSplitEvidence *> indexed;
  indexed.reserve(evidence.size());
  for (const auto & entry : evidence) {
    if (entry.hypothesis_id == invalid_hypothesis_id) {
      throw std::invalid_argument("Split evidence requires a valid hypothesis ID");
    }
    if (!known_ids.contains(entry.hypothesis_id)) {
      throw std::invalid_argument("Split evidence refers to an unknown hypothesis ID");
    }
    if (!indexed.emplace(entry.hypothesis_id, &entry).second) {
      throw std::invalid_argument("Duplicate split evidence for one hypothesis ID");
    }
  }
  return indexed;
}

}  // namespace

GaussianMixtureTracker::GaussianMixtureTracker(
  const GaussianMixture & initial_mixture,
  const GaussianMixtureTrackerConfig & config)
: config_(config),
  id_generator_(first_available_id(initial_mixture))
{
  validate_tracker_config(config_);
  validate_hypothesis_ids(initial_mixture);
  mixture_ = manage_gaussian_mixture(
    initial_mixture, bounded_management_config(config_)).mixture;
  validate_hypothesis_ids(mixture_);
}

GaussianMixtureTrackerResult GaussianMixtureTracker::update(
  const GaussianMixtureTrackerInput & input)
{
  GaussianMixtureTrackerResult result;
  result.update_sequence = update_sequence_ + 1U;

  GaussianMixture working = predict_gaussian_mixture(
    mixture_, input.motion, config_.prediction);

  if (input.observation.has_value()) {
    result.measurement_applied = true;
    result.measurement_update = update_gaussian_mixture(
      working, *input.observation, config_.update);
    working = result.measurement_update->mixture;

    LocalizationHealthEvidence health_evidence;
    health_evidence.measurement_update = &*result.measurement_update;
    result.pre_topology_health = evaluate_localization_health(
      working, health_evidence);
  } else {
    result.pre_topology_health = evaluate_localization_health(working);
  }

  const auto management_config = bounded_management_config(config_);
  result.pre_topology_management = manage_gaussian_mixture(
    working, management_config);

  result.merging = merge_gaussian_mixture_components(
    result.pre_topology_management.mixture,
    id_generator_,
    config_.merging);

  GaussianMixture post_topology = result.merging.mixture;

  if (config_.enable_splitting && !input.split_evidence.empty()) {
    if (input.source_particles.empty()) {
      throw std::invalid_argument(
              "Split evidence requires nonempty source particles");
    }

    const auto indexed_evidence = index_split_evidence(
      input.split_evidence,
      working);

    std::vector<GaussianComponentSplitEvidence> ordered_evidence;
    ordered_evidence.reserve(post_topology.components.size());

    std::unordered_set<HypothesisId> surviving_ids;
    surviving_ids.reserve(post_topology.components.size());

    for (const auto & component : post_topology.components) {
      surviving_ids.insert(component.provenance.id);
      const auto found = indexed_evidence.find(component.provenance.id);
      if (found == indexed_evidence.end()) {
        ordered_evidence.push_back(GaussianComponentSplitEvidence{
          {}, GaussianFitQuality{0.0, 0.0, 1.0}});
      } else {
        ordered_evidence.push_back(GaussianComponentSplitEvidence{
          found->second->source_indices,
          found->second->fit_quality});
      }
    }

    for (const auto & [id, unused] : indexed_evidence) {
      static_cast<void>(unused);
      if (!surviving_ids.contains(id)) {
        result.superseded_split_evidence_ids.push_back(id);
      }
    }
    std::sort(
      result.superseded_split_evidence_ids.begin(),
      result.superseded_split_evidence_ids.end());

    result.splitting = split_gaussian_mixture_components(
      input.source_particles,
      post_topology,
      ordered_evidence,
      id_generator_,
      config_.splitting);
    post_topology = result.splitting->mixture;
  }

  result.final_management = manage_gaussian_mixture(
    post_topology, management_config);
  result.component_limit_enforced =
    result.final_management.mixture.components.size() <=
    config_.maximum_component_count;

  if (!result.component_limit_enforced) {
    throw std::logic_error("Tracker failed to enforce maximum component count");
  }

  validate_hypothesis_ids(result.final_management.mixture);
  result.final_health = evaluate_localization_health(
    result.final_management.mixture);
  result.mixture = result.final_management.mixture;

  mixture_ = result.mixture;
  update_sequence_ = result.update_sequence;
  return result;
}

void GaussianMixtureTracker::reset(const GaussianMixture & mixture)
{
  validate_hypothesis_ids(mixture);
  mixture_ = manage_gaussian_mixture(
    mixture, bounded_management_config(config_)).mixture;
  validate_hypothesis_ids(mixture_);
  id_generator_ = HypothesisIdGenerator(first_available_id(mixture_));
  update_sequence_ = 0U;
}

const GaussianMixture & GaussianMixtureTracker::mixture() const noexcept
{
  return mixture_;
}

const GaussianMixtureTrackerConfig & GaussianMixtureTracker::config() const noexcept
{
  return config_;
}

std::size_t GaussianMixtureTracker::update_sequence() const noexcept
{
  return update_sequence_;
}

HypothesisId GaussianMixtureTracker::next_hypothesis_id() const noexcept
{
  return id_generator_.peek_next();
}

}  // namespace hybrid_localization
