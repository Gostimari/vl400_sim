cmake_minimum_required(VERSION 3.8)
project(vl400_description)

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
# gz-sim from the ROS 2 vendor packages (gz_sim_vendor & friends)
find_package(gz-sim REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

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

add_library(vl400_description_plugin SHARED
  src/livox_laser_simulation/livox_lidar_system.cpp
  src/livox_laser_simulation/livox_depth_sampler.cpp
)
# Installed as liblivox_laser_simulation.so, the library name referenced
# by the <plugin filename="liblivox_laser_simulation.so"> block of the
# livox_hap.xacro sensor.
set_target_properties(vl400_description_plugin PROPERTIES
  OUTPUT_NAME livox_laser_simulation)
target_include_directories(vl400_description_plugin PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vl400_description_plugin
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  ament_index_cpp::ament_index_cpp
  gz-sim::core
  gz-math::core
  gz-msgs::core
  gz-transport::core
  gz-plugin::gz-plugin-register
)

install(TARGETS vl400_description_plugin
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
    gz-sim::core
    gz-plugin::gz-plugin-register
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
  scripts/fake_state_publisher.py
  scripts/pub_GPS_XY_trator.py
  scripts/magnetometer_for_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
