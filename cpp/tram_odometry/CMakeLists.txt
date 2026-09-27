cmake_minimum_required(VERSION 3.8)
project(tram_odometry)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

option(TRAM_WITH_ONNXRUNTIME "Build the optional ONNX Runtime inference backend" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Custom message types shipped with the dataset. Without this package the
# /vehicle/* topics cannot even be deserialised, so it is a hard dependency.
find_package(tram_vehicle_msgs REQUIRED)
# Descriptor parsing for the learned corrector. yaml-cpp ships with ros-base
# (rcl_yaml_param_parser depends on it), so this adds no external dependency.
find_package(yaml-cpp REQUIRED)

set(TRAM_DEPS
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs diagnostic_msgs
  tf2 tf2_ros tf2_geometry_msgs ament_index_cpp tram_vehicle_msgs)

# ---------------------------------------------------------------- core library
add_library(tram_odometry_core SHARED
  src/params.cpp
  src/types.cpp
  src/geo.cpp
  src/signal_filter.cpp
  src/traction_model.cpp
  src/longitudinal_model.cpp
  src/slip_detector.cpp
  src/observer.cpp
  src/path_map.cpp
  src/estimator.cpp
  src/inference.cpp
  src/ml_corrector.cpp
)
target_include_directories(tram_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tram_odometry_core ${TRAM_DEPS})
target_link_libraries(tram_odometry_core yaml-cpp)

if(TRAM_WITH_ONNXRUNTIME)
  find_package(onnxruntime QUIET)
  if(onnxruntime_FOUND)
    target_compile_definitions(tram_odometry_core PUBLIC TRAM_WITH_ONNXRUNTIME)
    target_link_libraries(tram_odometry_core onnxruntime::onnxruntime)
    message(STATUS "tram_odometry: ONNX Runtime backend ENABLED")
  else()
    message(WARNING "tram_odometry: TRAM_WITH_ONNXRUNTIME=ON but onnxruntime was not "
                    "found; the corrector falls back to the built-in backends")
  endif()
endif()

# --------------------------------------------------------------------- nodes
add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node tram_odometry_core)
ament_target_dependencies(odometry_node ${TRAM_DEPS})

# Realtime budget evidence for criterion 4: no ROS, no bag, runs in CI.
add_executable(tram_bench src/bench_core.cpp)
target_link_libraries(tram_bench tram_odometry_core)
ament_target_dependencies(tram_bench ${TRAM_DEPS})

# ------------------------------------------------------------------- install
install(TARGETS tram_odometry_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS odometry_node tram_bench
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
# artifacts/ carries the route maps and lives INSIDE the package, so that
# `colcon build` produces a self-contained install tree: the node resolves the map
# through <share>/artifacts/route/..., which means it no longer depends on being
# launched from the repository root. The same maps are kept at the repository root
# because the Python tooling reads them from there.
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/artifacts")
  install(DIRECTORY artifacts DESTINATION share/${PROJECT_NAME})
else()
  message(WARNING "tram_odometry: no artifacts/ directory in the package; the route map "
                  "will not be installed and path_map.enable should be false")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core tram_odometry_core)
  ament_add_gtest(test_ml test/test_ml.cpp)
  target_link_libraries(test_ml tram_odometry_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(tram_odometry_core)
ament_export_dependencies(${TRAM_DEPS})
ament_package()
