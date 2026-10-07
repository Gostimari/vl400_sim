cmake_minimum_required(VERSION 3.8)
project(vl400_description)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
)

# Generator for the continuous track band mesh (belt_band.dae), attached
# statically to the track links of the URDF.
add_executable(belt_band_generator src/belt_band_generator.cc)
add_custom_command(TARGET belt_band_generator POST_BUILD
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/belt_band_generator
          ${CMAKE_CURRENT_SOURCE_DIR}/meshes/belt_band.dae
)

install(DIRECTORY
  launch
  worlds
  urdf
  meshes
  config
  models
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
