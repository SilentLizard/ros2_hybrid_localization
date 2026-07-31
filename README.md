# ros2_hybrid_localization
Combining AMCL and GMM for a more robust hybrid localization approach to enable quick convergence in global positioning and effective position tracking.
Exploring adaptive belief representations for mobile-robot localization. Particle filters are retained for global and strongly multimodal uncertainty, while compact Gaussian mixtures represent established pose hypotheses during steady-state tracking. The initial release clusters ROS 2 AMCL particle clouds into weighted SE(2) Gaussian components and provides visualization and benchmarking tools.
