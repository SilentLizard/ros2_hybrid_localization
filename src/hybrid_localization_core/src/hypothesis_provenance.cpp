#include "hybrid_localization_core/hypothesis_provenance.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

namespace hybrid_localization
{
namespace
{

[[nodiscard]] std::uint64_t next_generation(
  const HypothesisProvenance & first,
  const HypothesisProvenance & second)
{
  const std::uint64_t maximum = std::max(first.generation, second.generation);
  if (maximum == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Hypothesis generation overflows");
  }
  return maximum + 1U;
}

[[nodiscard]] std::uint64_t next_generation(
  const HypothesisProvenance & parent)
{
  if (parent.generation == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("Hypothesis generation overflows");
  }
  return parent.generation + 1U;
}

}  // namespace

HypothesisIdGenerator::HypothesisIdGenerator(const HypothesisId first_id)
: next_id_(first_id)
{
  if (first_id == invalid_hypothesis_id) {
    throw std::invalid_argument("First hypothesis ID must be nonzero");
  }
}

HypothesisId HypothesisIdGenerator::next()
{
  if (next_id_ == invalid_hypothesis_id) {
    throw std::overflow_error("Hypothesis ID sequence exhausted");
  }

  const HypothesisId result = next_id_;
  if (next_id_ == std::numeric_limits<HypothesisId>::max()) {
    next_id_ = invalid_hypothesis_id;
  } else {
    ++next_id_;
  }
  return result;
}

HypothesisId HypothesisIdGenerator::peek_next() const noexcept
{
  return next_id_;
}

bool has_hypothesis_id(const HypothesisProvenance & provenance) noexcept
{
  return provenance.id != invalid_hypothesis_id;
}

void validate_hypothesis_provenance(
  const HypothesisProvenance & provenance,
  const char * name)
{
  const std::string label = name == nullptr ? "Hypothesis provenance" : name;

  if (provenance.parent_count > provenance.parent_ids.size()) {
    throw std::invalid_argument(label + " has too many parents");
  }

  if (!has_hypothesis_id(provenance)) {
    if (provenance.parent_count != 0U || provenance.generation != 0U ||
      provenance.event != HypothesisProvenanceEvent::unassigned)
    {
      throw std::invalid_argument(
              label + " without an ID must be completely unassigned");
    }
    return;
  }

  if (provenance.event == HypothesisProvenanceEvent::unassigned) {
    throw std::invalid_argument(label + " with an ID must define an event");
  }

  for (std::size_t index = 0U; index < provenance.parent_ids.size(); ++index) {
    const bool active = index < provenance.parent_count;
    if (active && provenance.parent_ids[index] == invalid_hypothesis_id) {
      throw std::invalid_argument(label + " contains an invalid parent ID");
    }
    if (!active && provenance.parent_ids[index] != invalid_hypothesis_id) {
      throw std::invalid_argument(label + " contains an undeclared parent ID");
    }
    if (active && provenance.parent_ids[index] == provenance.id) {
      throw std::invalid_argument(label + " cannot be its own parent");
    }
  }

  if (provenance.parent_count == 0U && provenance.generation != 0U) {
    throw std::invalid_argument(label + " root generation must be zero");
  }
  if (provenance.parent_count > 0U && provenance.generation == 0U) {
    throw std::invalid_argument(label + " derived generation must be positive");
  }
}

HypothesisProvenance make_root_provenance(
  HypothesisIdGenerator & generator,
  const HypothesisProvenanceEvent event)
{
  if (event == HypothesisProvenanceEvent::unassigned ||
    event == HypothesisProvenanceEvent::merge ||
    event == HypothesisProvenanceEvent::split)
  {
    throw std::invalid_argument("Root provenance requires a root event");
  }

  return HypothesisProvenance{
    generator.next(),
    {invalid_hypothesis_id, invalid_hypothesis_id},
    0U,
    0U,
    event};
}

HypothesisProvenance make_merged_provenance(
  HypothesisIdGenerator & generator,
  const HypothesisProvenance & first,
  const HypothesisProvenance & second)
{
  validate_hypothesis_provenance(first, "First merge provenance");
  validate_hypothesis_provenance(second, "Second merge provenance");

  HypothesisProvenance result;
  result.id = generator.next();
  result.event = HypothesisProvenanceEvent::merge;
  result.generation = next_generation(first, second);

  if (has_hypothesis_id(first)) {
    result.parent_ids[result.parent_count++] = first.id;
  }
  if (has_hypothesis_id(second) && second.id != first.id) {
    result.parent_ids[result.parent_count++] = second.id;
  }

  if (result.parent_count == 0U) {
    result.generation = 0U;
  }
  return result;
}

HypothesisProvenance make_split_provenance(
  HypothesisIdGenerator & generator,
  const HypothesisProvenance & parent)
{
  validate_hypothesis_provenance(parent, "Split parent provenance");

  HypothesisProvenance result;
  result.id = generator.next();
  result.event = HypothesisProvenanceEvent::split;
  if (has_hypothesis_id(parent)) {
    result.parent_ids[0U] = parent.id;
    result.parent_count = 1U;
    result.generation = next_generation(parent);
  }
  return result;
}

}  // namespace hybrid_localization
