cmake_minimum_required(VERSION 3.16)
project(hybrid_localization_benchmark)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
# BEGIN AUTO BENCHMARK TOOLS
    scripts/belief_runtime_recorder.py
    scripts/localization_recorder.py
    scripts/resource_recorder.py
    scripts/run_benchmark.py
    scripts/summarize_run.py
# END AUTO BENCHMARK TOOLS
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
# BEGIN AUTO BENCHMARK PYTESTS
  ament_add_pytest_test(test_belief_runtime_recording test/test_belief_runtime_recording.py)
  ament_add_pytest_test(test_benchmark_run test/test_benchmark_run.py)
  ament_add_pytest_test(test_localization_recording test/test_localization_recording.py)
  ament_add_pytest_test(test_resource_recording test/test_resource_recording.py)
  ament_add_pytest_test(test_run_schema test/test_run_schema.py)
  ament_add_pytest_test(test_run_summary test/test_run_summary.py)
# END AUTO BENCHMARK PYTESTS
endif()

ament_package()
