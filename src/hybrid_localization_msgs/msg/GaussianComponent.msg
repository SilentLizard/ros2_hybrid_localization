# One weighted SE(2) Gaussian hypothesis.
# pose orientation represents yaw; roll and pitch are expected to be zero.
# covariance is row-major over [x, y, yaw].

geometry_msgs/Pose pose
float64[9] covariance
float64 weight
uint64 sample_count
hybrid_localization_msgs/HypothesisProvenance provenance
