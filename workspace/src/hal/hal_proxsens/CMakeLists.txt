cmake_minimum_required(VERSION 3.8)
project(hal_proxsens)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)
find_package(hal_proxsens_interfaces REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  hal_pigpio_interfaces
  hal_proxsens_interfaces
)

add_library(${PROJECT_NAME} OBJECT src/hal_proxsens.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node 
  src/hal_proxsens_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}> 
)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  #ament_lint_auto_find_test_dependencies()
  
  ament_add_gtest(${PROJECT_NAME}_test test/test_hal_proxsens.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()