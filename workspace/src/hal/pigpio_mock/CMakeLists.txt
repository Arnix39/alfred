cmake_minimum_required(VERSION 3.8)
project(pigpio_mock)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
  rclcpp
)

add_library(${PROJECT_NAME} SHARED src/pigpio_mock.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#############
## Install ##
#############
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()