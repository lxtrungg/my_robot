cmake_minimum_required(VERSION 3.0.2)
project(my_robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot_gazebo
  CATKIN_DEPENDS gazebo_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch models routes world
  DESTI
  
