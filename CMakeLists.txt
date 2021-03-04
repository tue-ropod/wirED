cmake_minimum_required(VERSION 2.8.3)
project(wired)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined") 

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++11 HAS_STD_CPP11_FLAG)
if(HAS_STD_CPP11_FLAG)
  add_compile_options(-std=c++11)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ed
  wire_core
  wire_msgs
  wire_features_lib
)

#find_package(wire_state_estimators
#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)

# Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#)

#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS}) 



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${Boost_INCLUDE_DIRS}
  LIBRARIES  ${PROJECT_NAME}
 #   CATKIN_DEPENDS geometry_msgs message_runtime
  CATKIN_DEPENDS
    wire_core wire_features_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(wired
  src/multiple_hypothesis_plugin.cpp
  include/multiple_hypothesis_plugin.h
  include/featureProperties_info.h
)

target_link_libraries(wired ed_core wire ${catkin_LIBRARIES})