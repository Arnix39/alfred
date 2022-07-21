cmake_minimum_required(VERSION 3.8)
project(hal_common)

find_package(ament_cmake REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  hal_pigpio_interfaces)

add_library(${PROJECT_NAME} SHARED
  src/hal_i2cRegistersServices.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#############
## Install ##
#############
install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)

ament_package()