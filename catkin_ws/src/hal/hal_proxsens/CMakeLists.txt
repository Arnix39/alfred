cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME hal_proxsens)
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
   hal_proxsensMsg.msg
 )

## Generate added messages and services with any dependencies listed here
  generate_messages(
   DEPENDENCIES 
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/hal_proxSens.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
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
catkin_add_gmock(${PROJECT_NAME}-test test/test_hal_proxSens.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
