cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME hal_imu)
project(${PROJECT_NAME})

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate added messages and services with any dependencies listed here
#  generate_messages(
#   DEPENDENCIES 
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hal
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${pigpio_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hal.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_node src/hal_imu.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${pigpio_LIBRAIRIES}
)


#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
