cmake_minimum_required(VERSION 3.10)
project(random_walk_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_library(random_walk SHARED src/RandomWalk.cc)
target_link_libraries(random_walk
  PUBLIC ignition-gazebo6::ignition-gazebo6
         ignition-plugin1::ignition-plugin1)

# Opcional pero cómodo si luego quieres usar filename="random_walk"
set_target_properties(random_walk PROPERTIES OUTPUT_NAME "random_walk")

# Instalar en lib/ (opcional)
install(TARGETS random_walk LIBRARY DESTINATION lib)
