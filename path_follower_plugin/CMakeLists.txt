cmake_minimum_required(VERSION 3.10.2)

project(path_follower_plugin)

# Find necessary Ignition libraries for Gazebo Fortress
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-math6 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build the plugin
add_library(path_follower_plugin SHARED PathFollower.cc)

# Link against the modern CMake targets. 
# This is more robust as it handles include directories and other dependencies automatically.
target_link_libraries(path_follower_plugin
  PRIVATE
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    ignition-math6::ignition-math6
)