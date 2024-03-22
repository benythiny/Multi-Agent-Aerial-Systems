cmake_minimum_required(VERSION 3.1.2)
project(task_02_formation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  )

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Og)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  LIBRARIES Task02Formation
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# wrapper

add_library(Task02Formation
  src/formation.cpp
  )

add_dependencies(Task02Formation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task02Formation
  ${catkin_LIBRARIES}
  )
