cmake_minimum_required(VERSION 2.8.3)
project(sef_pick_place)

add_compile_options(-std=c++11)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 moveit_core
 moveit_visual_tools
 moveit_ros_planning
 moveit_ros_planning_interface
 pluginlib
 geometric_shapes
 control_plugin)


## Declare a catkin package
catkin_package(
LIBRARIES
  interactivity_utils
INCLUDE_DIRS
  ${THIS_PACKAGE_INCLUDE_DIRS}
CATKIN_DEPENDS
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
DEPENDS
EIGEN3
)

## Build node
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(sef_pick_place src/sef_pick_place.cpp)
target_link_libraries(sef_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS sef_pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
