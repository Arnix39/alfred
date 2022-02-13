cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME hal_imu)
project(${PROJECT_NAME})

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  message_generation
  hal_pigpio
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate action servers in the 'action' folder
add_action_files(
  FILES
  hal_imuWriteDmp.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  actionlib_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib src/hal_imuDmpWritingServer.cpp)
add_dependencies(${PROJECT_NAME}_lib
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/hal_imu.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}DmpWritingServer_node src/hal_imuDmpWritingServer.cpp)
add_dependencies(${PROJECT_NAME}DmpWritingServer_node	${${PROJECT_NAME}_EXPORTED_TARGETS}	${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}DmpWritingServer_node
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
catkin_add_gmock(${PROJECT_NAME}-test test/test_hal_imuDmpWritingServer.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
endif()