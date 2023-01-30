cmake_minimum_required(VERSION 3.8)
project(hal_common)

find_package(ament_cmake REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)
find_package(hal_imu_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(common_utils REQUIRED)

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
  hal_imu_interfaces
  common_utils)

add_library(${PROJECT_NAME} SHARED
  src/hal_i2cRegistersServices.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

if(BUILD_TESTING)
  find_package(pigpio REQUIRED)
  add_library(${PROJECT_NAME}_mock SHARED
  src/mock/hal_i2cRegistersServices_mock.cpp)
  ament_target_dependencies(${PROJECT_NAME}_mock ${dependencies} pigpio)
endif()

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(pigpio REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test_hal_i2cRegistersServices.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies} pigpio)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()


#############
## Install ##
#############
install(
  DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  install(
    TARGETS ${PROJECT_NAME}_mock
    EXPORT ${PROJECT_NAME}_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)
else()
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)
endif()

ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)

ament_package()