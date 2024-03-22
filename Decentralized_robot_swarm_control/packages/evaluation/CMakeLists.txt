cmake_minimum_required(VERSION 3.1.2)
project(task_03_evaluation)

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
  nav_msgs
  task_03_swarm
  task_03_wrapper
  visualization_msgs
  )

catkin_package(
  LIBRARIES Task03SimulationTest Task03WeightingTest Task03BoidsTest
  CATKIN_DEPENDS roscpp nodelet mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs task_03_swarm task_03_wrapper visualization_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# SimulationTest

add_library(Task03SimulationTest
  src/simulation_test.cpp
  )

target_link_libraries(Task03SimulationTest
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03SimulationTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# WeightingTest

add_library(Task03WeightingTest
  src/weighting_test.cpp
  )

target_link_libraries(Task03WeightingTest
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03WeightingTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# BoidsTest

add_library(Task03BoidsTest
  src/boids_test.cpp
  )

target_link_libraries(Task03BoidsTest
  ${catkin_LIBRARIES}
  )

add_dependencies(Task03BoidsTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
