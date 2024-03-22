cmake_minimum_required(VERSION 3.2.0)
project(task_03_gazebo_resources)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")
# add_compile_options("-Og")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  mrs_lib
)

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS gazebo_ros
  CATKIN_DEPENDS roscpp mrs_lib geometry_msgs
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})


add_executable(target_mover
  src/target_mover.cpp
)
add_dependencies(target_mover
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_mover
   ${catkin_LIBRARIES}
)
