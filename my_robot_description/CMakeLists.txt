cmake_minimum_required(VERSION 3.0.2)
project(my_robot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  robot_state_publisher
  rviz
  joint_state_publisher_gui
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot_description
 CATKIN_DEPENDS urdf xacro robot_state_publisher rviz joint_state_publisher_gui
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY cad config launch meshs rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

