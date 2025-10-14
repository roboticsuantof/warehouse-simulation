cmake_minimum_required(VERSION 3.10)
project(actor_avoidance_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_library(actor_avoidance SHARED src/ActorAvoidance.cc)
target_link_libraries(actor_avoidance
  PUBLIC ignition-gazebo6::ignition-gazebo6
         ignition-plugin1::ignition-plugin1
         ignition-transport11::ignition-transport11
         ignition-msgs8::ignition-msgs8)

# Para que el fichero se llame libactor_avoidance.so
set_target_properties(actor_avoidance PROPERTIES OUTPUT_NAME "actor_avoidance")

install(TARGETS actor_avoidance LIBRARY DESTINATION lib)
