cmake_minimum_required(VERSION 3.1.2)
project(task_01_controller)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  )

set(CMAKE_CXX_STANDARD 17)
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
  LIBRARIES Task01Controller
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# wrapper

add_library(Task01Controller
  src/controller.cpp
  src/lkf.cpp
  src/utils.cpp
  )

add_dependencies(Task01Controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task01Controller
  ${catkin_LIBRARIES}
  )
