cmake_minimum_required(VERSION 3.1.2)
project(task_01_wrapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  mrs_msgs
  nav_msgs
  std_srvs
  nodelet
  message_generation
  mrs_uav_managers
  mrs_lib
  task_01_controller
  dynamic_reconfigure
  )

# messages for MPC_TRACKER
add_message_files(DIRECTORY msg FILES
  UserParams.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Og)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

generate_dynamic_reconfigure_options(
  cfg/wrapper.cfg
  cfg/user_params.cfg
  )

catkin_package(
  CATKIN_DEPENDS roscpp nodelet mrs_msgs mrs_uav_managers mrs_lib task_01_controller message_runtime std_srvs dynamic_reconfigure
  DEPENDS Eigen
  LIBRARIES Task01Wrapper Task01ParamServer Task01AutomaticStart Task01Spawner
  INCLUDE include
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# wrapper

add_library(Task01Wrapper
  src/wrapper.cpp
  )

add_dependencies(Task01Wrapper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task01Wrapper
  ${catkin_LIBRARIES}
  )

# param server

add_library(Task01ParamServer
  src/param_server.cpp
  )

add_dependencies(Task01ParamServer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task01ParamServer
  ${catkin_LIBRARIES}
  )

# automatic start

add_library(Task01AutomaticStart
  src/automatic_start.cpp
  )

add_dependencies(Task01AutomaticStart
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task01AutomaticStart
  ${catkin_LIBRARIES}
  )

# spawner

add_library(Task01Spawner
  src/spawner.cpp
  )

add_dependencies(Task01Spawner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Task01Spawner
  ${catkin_LIBRARIES}
  )
