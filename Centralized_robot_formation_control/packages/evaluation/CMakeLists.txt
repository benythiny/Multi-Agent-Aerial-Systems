cmake_minimum_required(VERSION 3.1.2)
project(task_02_evaluation)

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
  task_02_wrapper
  visualization_msgs
  )

catkin_package(
  LIBRARIES Task02SimulationTest Task02ReshapingTest Task02ReshapingDebug
  CATKIN_DEPENDS roscpp nodelet mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs task_02_formation task_02_wrapper visualization_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# ControllerTest

add_library(Task02SimulationTest
  src/simulation_test.cpp
  )

target_link_libraries(Task02SimulationTest
  ${catkin_LIBRARIES}
  )

add_dependencies(Task02SimulationTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# ReshapingTest

add_library(Task02ReshapingTest
  src/reshaping_test.cpp
  )

target_link_libraries(Task02ReshapingTest
  ${catkin_LIBRARIES}
  )

add_dependencies(Task02ReshapingTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# ReshapingDebug

add_library(Task02ReshapingDebug
  src/reshaping_debug.cpp
  )

target_link_libraries(Task02ReshapingDebug
  ${catkin_LIBRARIES}
  )

add_dependencies(Task02ReshapingDebug
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
