#pragma once

#include <span>

#include "hybrid_localization_core/types.hpp"

namespace hybrid_localization
{

[[nodiscard]] GaussianComponent fit_gaussian(
  std::span<const WeightedParticle> particles);

}  // namespace hybrid_localization
