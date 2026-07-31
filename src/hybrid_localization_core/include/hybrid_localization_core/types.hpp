#pragma once

#include <array>
#include <cstddef>

namespace hybrid_localization
{

//Pose2d data type as tuple including Euclidean space position in x y and rotation around z axis 
struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

//WeightedParticle data type for AMCL including Pose2d pose and particle weight for hypothesis rating
struct WeightedParticle
{
  Pose2d pose{};
  double weight{0.0};
};

//GaussianComponent data type for GMM including mean Pose2d, covariance matrix for the gaussian model and weight for hypothesis rating
struct GaussianComponent
{
  Pose2d mean{};

  // Row-major covariance for [x, y, yaw].
  std::array<double, 9> covariance{};

  double weight{0.0};
  std::size_t sample_count{0};
};

}  // namespace hybrid_localization
