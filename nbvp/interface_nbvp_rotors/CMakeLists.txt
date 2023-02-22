cmake_minimum_required(VERSION 2.8.3)
project(interface_nbvp_rotors)

find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  mav_msgs
  tf
  nbvplanner
)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
)

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration
  ${catkin_LIBRARIES}
)
