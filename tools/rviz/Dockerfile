FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-visualization-msgs \
    ros-jazzy-tf2-tools \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["rviz2"]
