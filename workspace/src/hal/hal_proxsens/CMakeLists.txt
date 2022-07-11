cmake_minimum_required(VERSION 3.8)
project(hal_proxsens)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)
find_package(hal_proxsens_interfaces REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)
add_executable(${PROJECT_NAME}_node src/hal_proxsens.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp hal_proxsens_interfaces hal_pigpio_interfaces)

#############
## Testing ##
#############
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  find_package(ament_cmake_gtest REQUIRED)
#  ament_lint_auto_find_test_dependencies()
  
#  ament_add_gtest(${PROJECT_NAME}_test test/test_hal_proxSens.cpp)
#  ament_target_dependencies(${PROJECT_NAME}_test rclcpp hal_proxsens_interfaces hal_pigpio_interfaces)
#endif()

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()