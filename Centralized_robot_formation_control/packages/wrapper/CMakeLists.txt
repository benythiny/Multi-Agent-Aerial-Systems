cmake_minimum_required(VERSION 3.1.2)
project(task_02_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Og)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_lib
  mrs_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  task_02_formation
  dynamic_reconfigure
  visualization_msgs
  message_generation
  mavros_msgs
  )

add_message_files(DIRECTORY msg FILES
  Diagnostics.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/wrapper.cfg
  )

catkin_package(
  LIBRARIES Task02Wrapper Task02Spawner Task02AutomaticStart
  CATKIN_DEPENDS roscpp nodelet mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs visualization_msgs message_runtime message_generation mavros_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Wrapper

add_library(Task02Wrapper
  src/wrapper.cpp
  )

target_link_libraries(Task02Wrapper
  ${catkin_LIBRARIES}
  )

add_dependencies(Task02Wrapper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Spawner

add_library(Task02Spawner
  src/spawner.cpp
  )

target_link_libraries(Task02Spawner
  ${catkin_LIBRARIES}
  )

add_dependencies(Task02Spawner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# AutomaticStart

add_library(Task02AutomaticStart
  src/automatic_start.cpp
  )

target_link_libraries(Task02AutomaticStart
  ${catkin_LIBRARIES}
  )

add_dependencies(Task02AutomaticStart
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
