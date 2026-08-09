cmake_minimum_required(VERSION 3.16)
project(hybrid_localization_ros LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hybrid_localization_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/amcl_particle_cloud_adapter.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  hybrid_localization_core
  nav2_msgs
  rclcpp
  std_msgs
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
)

add_executable(amcl_particle_cloud_adapter_smoke
  src/amcl_particle_cloud_adapter_smoke.cpp
)

target_link_libraries(amcl_particle_cloud_adapter_smoke
  ${PROJECT_NAME}
)

ament_target_dependencies(amcl_particle_cloud_adapter_smoke
  rclcpp
)

install(
  TARGETS
    ${PROJECT_NAME}
    amcl_particle_cloud_adapter_smoke
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  hybrid_localization_core
  nav2_msgs
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_amcl_particle_cloud_adapter
    test/test_amcl_particle_cloud_adapter.cpp
  )

  target_link_libraries(test_amcl_particle_cloud_adapter
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_amcl_particle_cloud_adapter
    geometry_msgs
    nav2_msgs
    rclcpp
  )
endif()

ament_package()
