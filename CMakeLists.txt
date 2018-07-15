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

#######################
## Find Dependencies ##
#######################
# Add search directory for CMake on OS X
list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/Modules)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(BUILD_GSTREAMER_PLUGIN "enable gstreamer plugin" "OFF")

option(BUILD_ROS_INTERFACE "Enable building ROS dependent plugins" "OFF")

## System dependencies are found with CMake's conventions
find_package(Boost 1.58 REQUIRED COMPONENTS system thread timer)
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
find_program(px4 REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(OpenCV REQUIRED)
if (BUILD_GSTREAMER_PLUGIN)
  find_package(GStreamer)
  if (GSTREAMER_FOUND)
    if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
      find_package (Qt4)
      include (${QT_USE_FILE})
    endif()
  endif()
endif()

pkg_check_modules(OGRE OGRE)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# for ROS subscribers and publishers
if (BUILD_ROS_INTERFACE)
  find_package(mavros REQUIRED)
  find_package(mavros_msgs REQUIRED)
  find_package(mav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(roscpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
endif()

# find MAVLink
#find_package(MAVLink)

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
  message(STATUS "catkin ENABLED")
  find_package(catkin REQUIRED)
  if (catkin_FOUND)
    catkin_package()
  else()
    message(FATAL_ERROR "catkin not found")
  endif()
else()
  message(STATUS "catkin DISABLED")
endif()

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message("Gazebo version: ${GAZEBO_VERSION}")
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


catkin_package(
  DEPENDS
    gazebo_ros
)

###########
## Build ##
###########

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wno-deprecated-declarations")

# include(EnableC++XX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -Wno-deprecated-declarations")

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
#  ${MAVLINK_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
  ${OpenCV_INCLUDE_DIRS}
# ${OpticalFlow_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  )

if (GSTREAMER_FOUND)
  include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
  )
endif()

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  )

if (GSTREAMER_FOUND)
  link_libraries(
    ${GSTREAMER_LIBRARIES}
  )
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_LIBRARY_DIRS}
  )


#--------------------#
# Message Generation #
#--------------------#

set(mav_msgs
  msgs/CommandMotorSpeed.proto
  msgs/MotorSpeed.proto
  )
set(nav_msgs msgs/Odometry.proto)
#set(physics_msgs msgs/Wind.proto)
set(std_msgs msgs/Int32.proto)
set(sensor_msgs
  msgs/Imu.proto
  msgs/IRLock.proto
  msgs/Float.proto
  msgs/Groundtruth.proto
  msgs/Range.proto
  msgs/SITLGps.proto
# OpticalFlow.proto
  )

PROTOBUF_GENERATE_CPP(MAV_PROTO_SRCS MAV_PROTO_HDRS ${mav_msgs})
PROTOBUF_GENERATE_CPP(NAV_PROTO_SRCS NAV_PROTO_HDRS ${nav_msgs})
#PROTOBUF_GENERATE_CPP(PHY_PROTO_SRCS PHY_PROTO_HDRS ${physics_msgs})
PROTOBUF_GENERATE_CPP(STD_PROTO_SRCS STD_PROTO_HDRS ${std_msgs})
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

add_library(mav_msgs SHARED ${MAV_PROTO_SRCS})
add_library(nav_msgs SHARED ${NAV_PROTO_SRCS})
#add_library(physics_msgs SHARED ${PHY_PROTO_SRCS})
add_library(std_msgs SHARED ${STD_PROTO_SRCS})
add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})


find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS}  ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
# ${MAVLINK_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
  ${OpenCV_INCLUDE_DIRS}
  )

############################################
########## FOR GAZEBO PLUGINS ##############
############################################
add_library(gazebo_ros_interface src/gazebo_ros_interface.cpp)
target_link_libraries(gazebo_ros_interface ${GAZEBO_LIBRARIES})
add_library(gazebo_motor_plugin src/gazebo_motor_plugin.cpp)
target_link_libraries(gazebo_motor_plugin ${GAZEBO_LIBRARIES})

#### From PX4/sitl_gazebo

link_libraries(mav_msgs nav_msgs std_msgs sensor_msgs)

add_library(gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
add_library(gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
#add_library(gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp)
add_library(gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
add_library(gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
#add_library(gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)

set(plugins
  gazebo_imu_plugin
#  gazebo_mavlink_interface
  gazebo_motor_model
  gazebo_multirotor_base_plugin
  #gazebo_wind_plugin
  )

foreach(plugin ${plugins})
  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})
endforeach()

foreach(plugin ${plugins})
  add_dependencies(${plugin} mav_msgs nav_msgs std_msgs sensor_msgs)
endforeach()


# FOR ROS NODES
# add_executable(gazebo_ros_interface src/gazebo_ros_interface.cpp)
# target_link_libraries(gazebo_ros_interface ${catkin_LIBRARIES} ${LibUSB_LIBRARIES})
# add_dependencies(gazebo_ros_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
