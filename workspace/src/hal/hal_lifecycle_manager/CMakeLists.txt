cmake_minimum_required(VERSION 3.8)
project(hal_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
)

add_library(${PROJECT_NAME} OBJECT src/hal_lifecycle_manager.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node 
  src/hal_lifecycle_manager_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()