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

# Python utilities packaged with the Isaac fixture. This includes Script Editor
# helpers plus simulator-independent scenario/world tooling. build.sh maintains
# this list; Python scripts importing rclpy are classified as ROS-host tools.
install(
  PROGRAMS
    # BEGIN AUTO ISAAC SUPPORT SCRIPTS
    scripts/build_heros_world.py
    scripts/build_localization_world.py
    scripts/change_isaac_world.py
    scripts/environment_fault_geometry.py
    scripts/fault_scheduler.py
    scripts/fault_specification.py
    scripts/fault_validation_matrix.py
    scripts/generate_heros_map.py
    scripts/generate_localization_map.py
    scripts/generate_world_layout.py
    scripts/heros_isaac_graph_helper.py
    scripts/inspect_heros_stage.py
    scripts/inspect_isaac_lidar.py
    scripts/isaac_environment_faults.py
    scripts/isaac_scenario_bridge.py
    scripts/lidar_fault_model.py
    scripts/materialize_world_scenario.py
    scripts/odometry_fault_model.py
    scripts/reset_heros_scenario.py
    scripts/scenario_runtime.py
    scripts/world_layout.py
    # END AUTO ISAAC SUPPORT SCRIPTS
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Host-side tools are installed under lib/${PROJECT_NAME} for `ros2 run`.
# build.sh classifies shell scripts and Python scripts importing rclpy here.
install(
  PROGRAMS
    # BEGIN AUTO ROS HOST TOOLS
    scripts/fault_scenario_acceptance.py
    scripts/lidar_fault_injector.py
    scripts/odometry_fault_injector.py
    scripts/reset_amcl_localization.py
    scripts/scenario_acceptance.py
    scripts/scenario_controller.py
    scripts/start_gamepad.sh
    scripts/start_gamepad_heros.sh
    scripts/start_observation_mode.sh
    scripts/validate_scan_map_alignment.py
    # END AUTO ROS HOST TOOLS
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    README.md
    THIRD_PARTY_NOTICE.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # This section is maintained by build.sh.
  # BEGIN AUTO PYTESTS
  ament_add_pytest_test(test_environment_fault_geometry test/test_environment_fault_geometry.py)
  ament_add_pytest_test(test_fault_scheduler test/test_fault_scheduler.py)
  ament_add_pytest_test(test_fault_specification test/test_fault_specification.py)
  ament_add_pytest_test(test_fault_validation_matrix test/test_fault_validation_matrix.py)
  ament_add_pytest_test(test_isaac_scenario_bridge test/test_isaac_scenario_bridge.py)
  ament_add_pytest_test(test_lidar_fault_model test/test_lidar_fault_model.py)
  ament_add_pytest_test(test_odometry_fault_model test/test_odometry_fault_model.py)
  ament_add_pytest_test(test_reset_heros_scenario test/test_reset_heros_scenario.py)
  ament_add_pytest_test(test_scenario_acceptance test/test_scenario_acceptance.py)
  ament_add_pytest_test(test_scenario_controller test/test_scenario_controller.py)
  ament_add_pytest_test(test_scenario_runtime test/test_scenario_runtime.py)
  ament_add_pytest_test(test_world_layout test/test_world_layout.py)
  ament_add_pytest_test(test_world_scenarios test/test_world_scenarios.py)
  # END AUTO PYTESTS
endif()

ament_package()
