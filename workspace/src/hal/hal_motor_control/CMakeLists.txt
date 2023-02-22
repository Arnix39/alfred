cmake_minimum_required(VERSION 3.8)
project(hal_motor_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)
find_package(hal_motor_control_interfaces REQUIRED)
find_package(common_utils REQUIRED)

###########
## Build ##
###########
include_directories(
  include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  hal_pigpio_interfaces
  hal_motor_control_interfaces
  common_utils)

add_library(${PROJECT_NAME}_motor_lib OBJECT src/hal_motor.cpp)
ament_target_dependencies(${PROJECT_NAME}_motor_lib ${dependencies})

add_library(${PROJECT_NAME} OBJECT src/hal_motor_control.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node
  src/hal_motor_control_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}_motor_lib>
  $<TARGET_OBJECTS:${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(pigpio REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_library(${PROJECT_NAME}_tests_helpers_lib OBJECT test/hal_motor_control_tests_helpers.cpp)
  ament_target_dependencies(${PROJECT_NAME}_tests_helpers_lib ${dependencies} pigpio)

  ament_add_gtest(${PROJECT_NAME}_motor_lib_test test/test_hal_motor.cpp)
  ament_target_dependencies(${PROJECT_NAME}_motor_lib_test ${dependencies} pigpio)
  target_link_libraries(${PROJECT_NAME}_motor_lib_test
    ${PROJECT_NAME}_motor_lib
    ${PROJECT_NAME}_tests_helpers_lib)

  ament_add_gtest(${PROJECT_NAME}_test
    test/test_hal_motor_control.cpp
    $<TARGET_OBJECTS:${PROJECT_NAME}_tests_helpers_lib>)
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies} pigpio)
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    ${PROJECT_NAME}_motor_lib
    ${PROJECT_NAME}_tests_helpers_lib)
endif()

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()