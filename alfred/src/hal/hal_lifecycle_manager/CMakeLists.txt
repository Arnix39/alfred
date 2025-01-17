cmake_minimum_required(VERSION 3.8)
project(hal_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(common_utils REQUIRED)
find_package(hal_camera REQUIRED)
find_package(hal_imu REQUIRED)
find_package(hal_motor_control REQUIRED)
find_package(hal_pigpio REQUIRED)
find_package(hal_pose_manager REQUIRED)
find_package(hal_proxsens REQUIRED)

###########
## Build ##
###########
include_directories(
  include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  common_utils
  hal_pigpio
  hal_camera
  hal_imu
  hal_motor_control
  hal_pose_manager
  hal_proxsens)

add_library(${PROJECT_NAME} OBJECT
  src/hal_lifecycle_manager.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node
  src/hal_lifecycle_manager_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test_hal_lifecycle_manager.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

#############
## Install ##
#############
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()