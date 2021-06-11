cmake_minimum_required(VERSION 3.0.2)
project(scavenger_interface)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp scavenger_hunt_msgs sensor_msgs darknet_ros_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ScavengerRosInterface src/scavengerRosInterface.cpp)
add_library(PassiveHunter src/passiveHunter.cpp)

add_executable(huntLauncher src/hunterNode.cpp)
add_dependencies(huntLauncher PassiveHunter ScavengerRosInterface)
target_link_libraries(huntLauncher ${catkin_LIBRARIES} ScavengerRosInterface PassiveHunter)
