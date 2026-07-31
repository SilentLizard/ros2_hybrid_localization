cmake_minimum_required(VERSION 3.16)
project(hybrid_localization_core LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# This section is maintained by build.sh.
# BEGIN AUTO SOURCES
set(HYBRID_LOCALIZATION_CORE_SOURCES
  src/adaptive_recovery_sampling.cpp
  src/gaussian_fit_quality.cpp
  src/gaussian_mixture.cpp
  src/gaussian_mixture_management.cpp
  src/gaussian_mixture_merging.cpp
  src/gaussian_mixture_prediction.cpp
  src/gaussian_mixture_update.cpp
  src/gaussian_statistics.cpp
  src/geometry.cpp
  src/global_particle_sampling.cpp
  src/particle_clustering.cpp
  src/particle_statistics.cpp
  src/recovery_sampling.cpp
)
# END AUTO SOURCES

add_library(${PROJECT_NAME}
  ${HYBRID_LOCALIZATION_CORE_SOURCES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # This section is maintained by build.sh.
  # BEGIN AUTO TESTS
  ament_add_gtest(test_adaptive_recovery_sampling
    test/test_adaptive_recovery_sampling.cpp
  )
  target_link_libraries(test_adaptive_recovery_sampling
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_gaussian_fit_quality
    test/test_gaussian_fit_quality.cpp
  )
  target_link_libraries(test_gaussian_fit_quality
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_gaussian_mixture
    test/test_gaussian_mixture.cpp
  )
  target_link_libraries(test_gaussian_mixture
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_gaussian_mixture_management
    test/test_gaussian_mixture_management.cpp
  )
  target_link_libraries(test_gaussian_mixture_management
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_gaussian_mixture_merging
    test/test_gaussian_mixture_merging.cpp
  )
  target_link_libraries(test_gaussian_mixture_merging
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_gaussian_mixture_prediction
    test/test_gaussian_mixture_prediction.cpp
  )
  target_link_libraries(test_gaussian_mixture_prediction
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_gaussian_mixture_update
    test/test_gaussian_mixture_update.cpp
  )
  target_link_libraries(test_gaussian_mixture_update
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_gaussian_statistics
    test/test_gaussian_statistics.cpp
  )
  target_link_libraries(test_gaussian_statistics
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_geometry
    test/test_geometry.cpp
  )
  target_link_libraries(test_geometry
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_global_particle_sampling
    test/test_global_particle_sampling.cpp
  )
  target_link_libraries(test_global_particle_sampling
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_particle_clustering
    test/test_particle_clustering.cpp
  )
  target_link_libraries(test_particle_clustering
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_particle_statistics
    test/test_particle_statistics.cpp
  )
  target_link_libraries(test_particle_statistics
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_recovery_sampling
    test/test_recovery_sampling.cpp
  )
  target_link_libraries(test_recovery_sampling
    ${PROJECT_NAME}
  )
  # END AUTO TESTS
endif()

ament_package()