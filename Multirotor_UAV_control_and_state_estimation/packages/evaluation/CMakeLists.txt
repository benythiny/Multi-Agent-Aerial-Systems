cmake_minimum_required(VERSION 3.1.2)
project(task_01_evaluation)

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
  task_01_controller
  )

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES Task01KalmanTest Task01Controllertest
  CATKIN_DEPENDS roscpp nodelet mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# Kalman

add_library(Task01KalmanTest
  src/kalman_test.cpp
  )

target_link_libraries(Task01KalmanTest
  ${catkin_LIBRARIES}
  )

add_dependencies(Task01KalmanTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# ControllerTest

add_library(Task01ControllerTest
  src/controller_test.cpp
  )

target_link_libraries(Task01ControllerTest
  ${catkin_LIBRARIES}
  )

add_dependencies(Task01ControllerTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
