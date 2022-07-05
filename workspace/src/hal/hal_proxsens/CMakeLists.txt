cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME hal_proxsens)
project(${PROJECT_NAME})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hal_pigpio REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/HalProxsens.msg
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/hal_proxsens.cpp)
ament_target_dependencies(${PROJECT_NAME}_node hal_pigpio rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}") 

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()