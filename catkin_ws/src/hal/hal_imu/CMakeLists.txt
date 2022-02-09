cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME hal_imu)
project(${PROJECT_NAME})

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib_msgs
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
  DIRECTORY action
  FILES hal_imuWriteDMP.action
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

add_executable(${PROJECT_NAME}_node src/hal_imu.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}Init_node src/hal_imuInit.cpp)
target_link_libraries(${PROJECT_NAME}Init_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
