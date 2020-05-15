cmake_minimum_required(VERSION 2.8.3)
project(simple_gestures)

## Compile as C++11
# add_compile_options(-std=c++11)


if(CATKIN_DEVEL_PREFIX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

endif()


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_library(k4a_LIB k4a REQUIRED)
find_library(k4abt_LIB k4abt REQUIRED)


if(COMMAND catkin_package)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/common/angle_converter.cpp
  src/common/vector3.cpp
  src/kinect/azure_kinect_gestures.cpp
  src/kinect/azure_kinect.cpp
  src/gestures/gesture.cpp
  src/gestures/gesture_group.cpp
  src/gestures/gesture_item.cpp
  src/gestures/gestures_engine.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${k4a_LIB}
  ${k4abt_LIB}
)