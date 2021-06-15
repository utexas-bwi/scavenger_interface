cmake_minimum_required(VERSION 3.0.2)
project(scavenger_interface)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp scavenger_hunt_msgs sensor_msgs darknet_ros_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(scavenger_ros_interface src/scavenger_ros_interface.cpp)
add_library(passive_hunter src/passive_hunter.cpp)

add_executable(hunter_launcher src/hunter_launcher.cpp)
add_dependencies(hunter_launcher passive_hunter scavenger_ros_interface)
target_link_libraries(hunter_launcher ${catkin_LIBRARIES} scavenger_ros_interface passive_hunter)
