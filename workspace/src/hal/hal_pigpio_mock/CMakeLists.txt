cmake_minimum_required(VERSION 3.8)
project(hal_pigpio_mock)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ../include
)

set(dependencies
rclcpp
hal_pigpio_interfaces
)

add_executable(${PROJECT_NAME} 
  src/hal_pigpio_mock.cpp 
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()