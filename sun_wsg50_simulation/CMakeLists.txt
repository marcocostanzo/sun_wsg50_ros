cmake_minimum_required(VERSION 2.8.3)
project(sun_wsg50_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sun_ros_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(grasp_force_visualizer src/grasp_force_visualizer_node.cpp)
target_link_libraries(grasp_force_visualizer ${catkin_LIBRARIES})
