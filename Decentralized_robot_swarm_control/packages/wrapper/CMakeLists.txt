cmake_minimum_required(VERSION 3.1.2)
project(task_03_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
# add_definitions(-Og)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_lib
  mrs_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
  task_03_swarm
  mavros_msgs
  gazebo_ros
  gazebo_msgs
  )

find_package(gazebo REQUIRED)

# messages for MPC_TRACKER
add_message_files(DIRECTORY msg FILES
  SwarmMemberState.msg
  UserParams.msg
  Diagnostics.msg
  CorruptMeasurements.msg
  )

generate_dynamic_reconfigure_options(
  cfg/param_server.cfg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
  )

set(LIBRARIES
  Task03WrapperBoids Task03WrapperHuntTheRobot Task03Spawner Task03ParamServer Task03AutomaticStart Task03Randomizer Task03CollisionChecker
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp nodelet mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs message_runtime task_03_swarm mavros_msgs visualization_msgs gazebo_ros gazebo_msgs
  INCLUDE_DIRS include
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${GAZEBO_INCLUDE_DIRS}
  )

# Wrapper for the Boids task
add_library(Task03WrapperBoids
  src/wrapper_boids.cpp
  )

target_link_libraries(Task03WrapperBoids
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03WrapperBoids
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Wrapper for the Hunt-the-Robot task
add_library(Task03WrapperHuntTheRobot
  src/wrapper_hunt_the_robot.cpp
  )

target_link_libraries(Task03WrapperHuntTheRobot
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03WrapperHuntTheRobot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Spawner

add_library(Task03Spawner
  src/spawner.cpp
  )

target_link_libraries(Task03Spawner
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03Spawner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# ParamServer

add_library(Task03ParamServer
  src/param_server.cpp
  )

target_link_libraries(Task03ParamServer
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03ParamServer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Plotter

add_library(Task03Plotter
  src/plotter.cpp
  )

target_link_libraries(Task03Plotter
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03Plotter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# AutomaticStart

add_library(Task03AutomaticStart
  src/automatic_start.cpp
  )

target_link_libraries(Task03AutomaticStart
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03AutomaticStart
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Randomizer

add_library(Task03Randomizer
  src/randomizer.cpp
  )

target_link_libraries(Task03Randomizer
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03Randomizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# CollisionChecker

add_library(Task03CollisionChecker
  src/collision_checker.cpp
  )

target_link_libraries(Task03CollisionChecker
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  )

add_dependencies(Task03CollisionChecker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## --------------------------------------------------------------
## |                            INSTALL                         |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
