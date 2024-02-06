cmake_minimum_required(VERSION 3.8)
project(hal_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(common_utils REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
  rclcpp_lifecycle
  common_utils
  OpenCV
  cv_bridge
  sensor_msgs
  std_msgs)

add_library(${PROJECT_NAME} SHARED src/hal_camera.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node
  src/hal_camera_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test_hal_camera.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()