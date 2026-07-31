#pragma once

#include <span>
#include <vector>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{
 
[[nodiscard]] std::vector<WeightedParticle> normalize_weights(
  std::span<const WeightedParticle> particles);

[[nodiscard]] Pose2d weighted_mean(
  std::span<const WeightedParticle> particles);

[[nodiscard]] double effective_sample_size(
  std::span<const WeightedParticle> particles);

}  // namespace hybrid_localization
