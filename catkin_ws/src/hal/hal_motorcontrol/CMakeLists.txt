cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME hal_motorcontrol)
project(${PROJECT_NAME})

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  hal_pigpio
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  hal_motorcontrolMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  roscpp
)

###########
## Build ##
###########
include_directories(
  include
  include/motor
  include/motorControl
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_motor_lib OBJECT src/hal_motor.cpp)
add_dependencies(${PROJECT_NAME}_motor_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node 
  src/hal_motorControl.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}_motor_lib>
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
#catkin_add_gmock(${PROJECT_NAME}-test test/test_hal_motorControl.cpp)
#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES}
#endif()
