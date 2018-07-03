cmake_minimum_required(VERSION 2.8.8)
project(hiperlab_gazebo)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  hiperlab_common
  hiperlab_components
  hiperlab_hardware
  hiperlab_rostools
  roscpp
  rospy
)


catkin_package(
  DEPENDS
    gazebo_ros
#  INCLUDE_DIRS include
#  LIBRARIES hiperlab_gazebo
#  CATKIN_DEPENDS gazebo_plugins gazebo_ros hiperlab_common hiperlab_components hiperlab_hardware hiperlab_rostools roscpp rospy
)

# Protobuf Stuff
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS}  ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

# FOR GAZEBO PLUGINS
add_library(gazebo_ros_interface src/gazebo_ros_interface.cpp)
target_link_libraries(gazebo_ros_interface ${GAZEBO_LIBRARIES})
add_library(gazebo_motor_plugin src/gazebo_motor_plugin.cpp)
target_link_libraries(gazebo_motor_plugin ${GAZEBO_LIBRARIES})


# FOR ROS NODES
# add_executable(gazebo_ros_interface src/gazebo_ros_interface.cpp)
# target_link_libraries(gazebo_ros_interface ${catkin_LIBRARIES} ${LibUSB_LIBRARIES})
# add_dependencies(gazebo_ros_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
