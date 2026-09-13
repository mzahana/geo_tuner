cmake_minimum_required(VERSION 3.8)
project(geo_tuner)

# The round-end identification is a grid search over the whole session: an
# unoptimised build is several times slower on the vehicle computer.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mav_controllers_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# Optional: supplies the conductor's OFFBOARD supervision and the panels'
# PX4 flight-mode readout. Everything else builds and runs without it.
find_package(mavros_msgs QUIET)

# ---------------------------------------------------------------------
# Core identification / design math. No ROS: this is what the unit tests
# and the offline CLI link against.
add_library(geo_tuner_core SHARED
  src/core/gain_design.cpp
  src/core/gain_yaml.cpp
  src/core/yaml_double.cpp
  src/core/least_squares.cpp
  src/core/step_fit.cpp
  src/core/loop_fit.cpp
  src/core/first_order_fit.cpp
  src/core/safety.cpp
  src/core/aggregate.cpp
  src/core/accel_loop_id.cpp
  src/core/loop_design.cpp
  src/core/session_recording.cpp
  src/tuning_schedule.cpp
)
target_include_directories(geo_tuner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(geo_tuner_core Eigen3::Eigen ${YAML_CPP_LIBRARIES})
target_include_directories(geo_tuner_core PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# ---------------------------------------------------------------------
# Nodes
add_library(geo_tuner_nodes SHARED
  src/tuning_conductor.cpp
  src/quad_sim.cpp
)
target_include_directories(geo_tuner_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(geo_tuner_nodes geo_tuner_core)
ament_target_dependencies(geo_tuner_nodes
  rclcpp rcl_interfaces nav_msgs geometry_msgs trajectory_msgs std_msgs
  std_srvs diagnostic_msgs mav_controllers_ros
)
if(mavros_msgs_FOUND)
  ament_target_dependencies(geo_tuner_nodes mavros_msgs)
  target_compile_definitions(geo_tuner_nodes PUBLIC GEO_TUNER_HAVE_MAVROS_MSGS)
else()
  message(STATUS "mavros_msgs not found: the conductor cannot supervise OFFBOARD")
endif()

add_executable(tuning_conductor src/nodes/tuning_conductor_node.cpp)
target_link_libraries(tuning_conductor geo_tuner_nodes)

add_executable(quad_sim src/nodes/quad_sim_node.cpp)
target_link_libraries(quad_sim geo_tuner_nodes)

# Offline gain designer (was the geo-tuner-design console script).
add_executable(geo-tuner-design src/nodes/design_gains_main.cpp)
target_link_libraries(geo-tuner-design geo_tuner_core)

# Offline identification + gain decision on a session recording (the
# conductor's own CSV, or one exported from an older bag).
add_executable(geo-tuner-identify src/nodes/identify_session_main.cpp)
target_link_libraries(geo-tuner-identify geo_tuner_core)

install(TARGETS tuning_conductor quad_sim geo-tuner-design geo-tuner-identify
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS geo_tuner_core geo_tuner_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------
# RViz field panels. Built only where RViz is installed, so a vehicle-side
# build (Jetson, docker without a GUI stack) is unaffected by folding them
# into this package.
find_package(rviz_common QUIET)
find_package(pluginlib QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets)
if(rviz_common_FOUND AND pluginlib_FOUND AND Qt5_FOUND)
  # Panels are QObjects with signals/slots; moc must run over the headers.
  set(CMAKE_AUTOMOC ON)

  add_library(geo_tuner_panels SHARED
    src/rviz/namespace_selector.cpp
    src/rviz/sparkline_widget.cpp
    src/rviz/controller_health_panel.cpp
    src/rviz/gain_panel.cpp
    src/rviz/tuner_panel.cpp
    src/rviz/geo_field_panel.cpp
    src/rviz/trajectory_test_panel.cpp
    include/geo_tuner/rviz/namespace_selector.hpp
    include/geo_tuner/rviz/sparkline_widget.hpp
    include/geo_tuner/rviz/controller_health_panel.hpp
    include/geo_tuner/rviz/gain_panel.hpp
    include/geo_tuner/rviz/tuner_panel.hpp
    include/geo_tuner/rviz/geo_field_panel.hpp
    include/geo_tuner/rviz/trajectory_test_panel.hpp
  )
  target_include_directories(geo_tuner_panels PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(geo_tuner_panels Qt5::Widgets)
  ament_target_dependencies(geo_tuner_panels
    rclcpp rviz_common pluginlib diagnostic_msgs std_srvs std_msgs
  )
  if(mavros_msgs_FOUND)
    ament_target_dependencies(geo_tuner_panels mavros_msgs)
    target_compile_definitions(geo_tuner_panels PRIVATE GEO_TUNER_HAVE_MAVROS_MSGS)
  else()
    message(STATUS "mavros_msgs not found: panels build without the PX4 flight-mode readout")
  endif()

  pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
  install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
  install(TARGETS geo_tuner_panels
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_dependencies(rviz_common pluginlib Qt5)
else()
  message(STATUS
    "rviz_common/pluginlib/Qt5 not found: building geo_tuner without the RViz panels")
endif()

# ---------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Shipped as share/geo_tuner/rviz so the launch files' paths are unchanged.
install(DIRECTORY rviz_config/ DESTINATION share/${PROJECT_NAME}/rviz)

# The only Python left: the offline ulog analyser (pyulog) and the 3D
# tracking-trail republisher.
install(PROGRAMS scripts/geo-tuner-hover scripts/tracking_viz.py scripts/geo-tuner-bag-export
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geo_tuner_core
    test/test_core.cpp
    test/test_schedule.cpp
    test/test_identification.cpp
  )
  if(TARGET test_geo_tuner_core)
    target_link_libraries(test_geo_tuner_core geo_tuner_core)
    # Field-flight session recordings (test_identification.cpp, FieldFlights).
    target_compile_definitions(test_geo_tuner_core PRIVATE
      GEO_TUNER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rcl_interfaces nav_msgs geometry_msgs trajectory_msgs std_msgs
  std_srvs diagnostic_msgs mav_controllers_ros Eigen3 yaml-cpp
)

ament_package()
