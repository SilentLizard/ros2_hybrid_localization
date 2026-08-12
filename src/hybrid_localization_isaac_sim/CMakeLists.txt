cmake_minimum_required(VERSION 3.16)
project(hybrid_localization_isaac_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs maps robot
  DESTINATION share/${PROJECT_NAME}
)

# Isaac Script Editor utilities are assets rather than ROS executables.
install(
  PROGRAMS
    scripts/build_heros_world.py
    scripts/generate_heros_map.py
    scripts/heros_isaac_graph_helper.py
    scripts/inspect_heros_stage.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Host-side utilities are available through `ros2 run`.
install(
  PROGRAMS
    scripts/start_gamepad.sh
    scripts/start_observation_mode.sh
    scripts/validate_scan_map_alignment.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    README.md
    THIRD_PARTY_NOTICE.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
