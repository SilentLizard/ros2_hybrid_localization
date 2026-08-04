# Bounded SE(2) Gaussian-mixture belief.
# sum(components.weight) + discarded_weight should equal 1 within tolerance.

std_msgs/Header header
uint64 update_sequence
hybrid_localization_msgs/GaussianComponent[] components
float64 discarded_weight
