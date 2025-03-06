cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualization)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

## Find system dependencies
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveTrajectory.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    nav_msgs 
    geometry_msgs 
    visualization_msgs 
    tf2 
    tf2_ros 
    tf2_geometry_msgs 
    message_runtime
  DEPENDS yaml-cpp JSONCPP
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

## Declare a C++ executable for trajectory_saver
add_executable(trajectory_saver src/trajectory_saver.cpp)
add_dependencies(trajectory_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_saver
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

## Declare a C++ executable for trajectory_reader
add_executable(trajectory_reader src/trajectory_reader.cpp)
add_dependencies(trajectory_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_reader
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

## Install executables and/or libraries
install(TARGETS trajectory_saver trajectory_reader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install RViz configuration files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

## Install data files
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)