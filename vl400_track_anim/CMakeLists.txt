cmake_minimum_required(VERSION 3.10)
project(vl400_track_anim)

find_package(ament_cmake REQUIRED)

# gz-sim from the ROS 2 vendor package (gz_sim_vendor)
find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register loader)

add_library(vl400-track-visual-system SHARED src/TrackVisual.cc)
target_compile_features(vl400-track-visual-system PUBLIC cxx_std_17)
target_link_libraries(vl400-track-visual-system
  PRIVATE
    gz-sim::core
    gz-plugin::gz-plugin-register
)

install(TARGETS vl400-track-visual-system
  LIBRARY DESTINATION lib
)

ament_package()
