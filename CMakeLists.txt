cmake_minimum_required(VERSION 2.8.3)
project(moveit_vive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# file(GLOB SHARED_SRC_FILES
#   ${SHARED_SRC_DIR}/*.cpp
#   ${SHARED_SRC_DIR}/*.h
# )

# FIND_LIBRARY(EIGEN_LIBRARY Eigen ~/libraries/c++/eigen)
# INCLUDE_DIRECTORIES(~/libraries/c++/eigen)
# LINK_DIRECTORIES(~/libraries/c++/eigen)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rospy
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(moveit_controller
  src/controller/moveit_controller.cpp
  src/controller/robotarm.cpp
)

target_link_libraries(
  moveit_controller 
  ${catkin_LIBRARIES} 
)
