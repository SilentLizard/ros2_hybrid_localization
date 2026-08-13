cmake_minimum_required(VERSION 3.16)
project(hybrid_localization_isaac_sim)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

install(
  DIRECTORY config docs maps robot
  DESTINATION share/${PROJECT_NAME}
)

# Isaac Script Editor utilities are assets rather than ROS executables.
install(
  PROGRAMS
    scripts/build_heros_world.py
    scripts/build_localization_world.py
    scripts/change_isaac_world.py
    scripts/generate_heros_map.py
    scripts/generate_localization_map.py
    scripts/generate_world_layout.py
    scripts/materialize_world_scenario.py
    scripts/heros_isaac_graph_helper.py
    scripts/inspect_heros_stage.py
    scripts/inspect_isaac_lidar.py
    scripts/world_layout.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Host-side utilities are available through `ros2 run`.
install(
  PROGRAMS
    scripts/start_gamepad.sh
    scripts/start_observation_mode.sh
    scripts/reset_amcl_localization.py
    scripts/validate_scan_map_alignment.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    README.md
    THIRD_PARTY_NOTICE.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_pytest_test(test_world_layout test/test_world_layout.py)
  ament_add_pytest_test(test_world_scenarios test/test_world_scenarios.py)
endif()

ament_package()
