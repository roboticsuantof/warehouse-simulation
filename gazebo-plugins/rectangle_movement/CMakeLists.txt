cmake_minimum_required(VERSION 3.10)
project(RectangleMover)

find_package(ignition-gazebo6 REQUIRED)

add_library(rectangle_movement SHARED src/RectangleMover.cc)
target_link_libraries(rectangle_movement
  ignition-gazebo6::core
)
