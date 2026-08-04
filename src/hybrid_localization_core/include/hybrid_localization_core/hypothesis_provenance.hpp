#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hybrid_localization
{

using HypothesisId = std::uint64_t;
inline constexpr HypothesisId invalid_hypothesis_id{0U};

enum class HypothesisProvenanceEvent : std::uint8_t
{
  unassigned,
  particle_cluster,
  merge,
  split
};

/// Compact direct-lineage metadata for one Gaussian hypothesis.
///
/// Identity-preserving operations such as prediction, correction, sorting, and
/// normalization copy this value unchanged. Merge and split operations allocate
/// new IDs and record their direct parents. Full ancestry can therefore be
/// reconstructed from emitted tracker diagnostics without storing an unbounded
/// history in every component.
struct HypothesisProvenance
{
  HypothesisId id{invalid_hypothesis_id};
  std::array<HypothesisId, 2> parent_ids{
    invalid_hypothesis_id,
    invalid_hypothesis_id};
  std::uint8_t parent_count{0U};
  std::uint64_t generation{0U};
  HypothesisProvenanceEvent event{HypothesisProvenanceEvent::unassigned};
};

/// Monotonic nonzero hypothesis-ID allocator.
class HypothesisIdGenerator
{
public:
  explicit HypothesisIdGenerator(HypothesisId first_id = 1U);

  [[nodiscard]] HypothesisId next();
  [[nodiscard]] HypothesisId peek_next() const noexcept;

private:
  HypothesisId next_id_{1U};
};

[[nodiscard]] bool has_hypothesis_id(
  const HypothesisProvenance & provenance) noexcept;

void validate_hypothesis_provenance(
  const HypothesisProvenance & provenance,
  const char * name = "Hypothesis provenance");

[[nodiscard]] HypothesisProvenance make_root_provenance(
  HypothesisIdGenerator & generator,
  HypothesisProvenanceEvent event = HypothesisProvenanceEvent::particle_cluster);

[[nodiscard]] HypothesisProvenance make_merged_provenance(
  HypothesisIdGenerator & generator,
  const HypothesisProvenance & first,
  const HypothesisProvenance & second);

[[nodiscard]] HypothesisProvenance make_split_provenance(
  HypothesisIdGenerator & generator,
  const HypothesisProvenance & parent);

}  // namespace hybrid_localization
