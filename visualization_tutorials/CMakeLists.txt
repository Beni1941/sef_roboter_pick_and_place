cmake_minimum_required(VERSION 2.8.3)
project(visualization_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED interactive_markers roscpp visualization_msgs tf)


catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(simple_marker src/simple_marker.cpp)
target_link_libraries(simple_marker
   ${catkin_LIBRARIES}
)

add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/basic_controls.py
  scripts/simple_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS
  simple_marker
  basic_controls
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
