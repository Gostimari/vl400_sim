cmake_minimum_required(VERSION 3.8)
project(vl400_sim)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Dependencies of the Livox lidar simulation plugin built below
# (sources in src/livox_laser_simulation, messages in msg/).
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
# gz-sim from the ROS 2 vendor packages. Lyrical ships the unsuffixed
# configs (gz-sim, gz-math, ...); Jazzy ships the versioned ones
# (gz-sim8, gz-math7, gz-msgs10, gz-transport13, gz-plugin2, with the
# plugin register split into its own gz-plugin2-register config).
# Detect once and map the link targets.
find_package(gz-sim QUIET)
if(gz-sim_FOUND)
  set(GZ_SIM_TARGET gz-sim::core)
  set(GZ_MATH_TARGET gz-math::core)
  set(GZ_MSGS_TARGET gz-msgs::core)
  set(GZ_TRANSPORT_TARGET gz-transport::core)
  set(GZ_PLUGIN_TARGET gz-plugin::gz-plugin-register)
else()
  find_package(gz-sim8 REQUIRED)
  find_package(gz-math7 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-plugin2 REQUIRED)
  find_package(gz-plugin2-register REQUIRED)
  set(GZ_SIM_TARGET gz-sim8::core)
  set(GZ_MATH_TARGET gz-math7::core)
  set(GZ_MSGS_TARGET gz-msgs10::core)
  set(GZ_TRANSPORT_TARGET gz-transport13::core)
  set(GZ_PLUGIN_TARGET gz-plugin2::register)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

# Generator for the continuous track band mesh (belt_band.dae), attached
# statically to the track links of the URDF.
add_executable(belt_band_generator src/belt_band_generator.cc)
add_custom_command(TARGET belt_band_generator POST_BUILD
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/belt_band_generator
          ${CMAKE_CURRENT_SOURCE_DIR}/meshes/belt_band.dae
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_library(vl400_sim_plugin SHARED
  src/livox_laser_simulation/livox_lidar_system.cpp
  src/livox_laser_simulation/livox_depth_sampler.cpp
)
# Installed as liblivox_laser_simulation.so, the library name referenced
# by the <plugin filename="liblivox_laser_simulation.so"> block of the
# livox_hap.xacro sensor.
set_target_properties(vl400_sim_plugin PROPERTIES
  OUTPUT_NAME livox_laser_simulation)
target_include_directories(vl400_sim_plugin PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vl400_sim_plugin
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  ament_index_cpp::ament_index_cpp
  ${GZ_SIM_TARGET}
  ${GZ_MATH_TARGET}
  ${GZ_MSGS_TARGET}
  ${GZ_TRANSPORT_TARGET}
  ${GZ_PLUGIN_TARGET}
)

install(TARGETS vl400_sim_plugin
  LIBRARY DESTINATION lib
)

# Tread plate animation system (fused in from the former
# vl400_track_anim package). Installed as libvl400-track-visual-system.so,
# the library name referenced by the
# <plugin filename="vl400-track-visual-system"> block of urdf/vl400.xacro.
add_library(vl400-track-visual-system SHARED src/track_visual/TrackVisual.cc)
target_compile_features(vl400-track-visual-system PUBLIC cxx_std_17)
target_link_libraries(vl400-track-visual-system
  PRIVATE
    ${GZ_SIM_TARGET}
    ${GZ_PLUGIN_TARGET}
)

install(TARGETS vl400-track-visual-system
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  launch
  worlds
  urdf
  meshes
  config
  models
  scan_patterns
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/magnetometer_for_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
