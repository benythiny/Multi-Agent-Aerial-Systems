cmake_minimum_required(VERSION 3.1.2)
project(task_03_swarm)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  )

add_definitions(-Wall)
add_definitions(-Wextra)
# add_definitions(-Og)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  LIBRARIES Task03Swarm Task03Boids
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# Decentralized formation control
add_library(Task03Swarm
  src/swarm.cpp
  )

add_dependencies(Task03Swarm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task03Swarm
  ${catkin_LIBRARIES}
  )

# Boids
add_library(Task03Boids
  src/boids.cpp
  )

add_dependencies(Task03Boids
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task03Boids
  ${catkin_LIBRARIES}
  )
