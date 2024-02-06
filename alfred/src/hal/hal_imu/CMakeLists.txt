cmake_minimum_required(VERSION 3.8)
project(hal_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)
find_package(hal_imu_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hal_common REQUIRED)
find_package(common_utils REQUIRED)

###########
## Build ##
###########
include_directories(
  include)

set(dependencies
  rclcpp_action
  rclcpp_lifecycle
  hal_pigpio_interfaces
  hal_imu_interfaces
  sensor_msgs
  hal_common
  common_utils)

set(test_dependencies
  pigpio
)

add_library(${PROJECT_NAME} SHARED src/hal_imu.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_library(${PROJECT_NAME}I2cInit SHARED src/hal_imuI2cInit.cpp)
ament_target_dependencies(${PROJECT_NAME}I2cInit ${dependencies})

add_library(${PROJECT_NAME}DmpWritingServer SHARED src/hal_imuDmpWritingServer.cpp)
ament_target_dependencies(${PROJECT_NAME}DmpWritingServer ${dependencies})

add_executable(${PROJECT_NAME}_node
  src/hal_imu_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

add_executable(${PROJECT_NAME}I2cInit_node
  src/hal_imuI2cInit_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}I2cInit>)
ament_target_dependencies(${PROJECT_NAME}I2cInit_node ${dependencies})

add_executable(${PROJECT_NAME}DmpWritingServer_node
  src/hal_imuDmpWritingServer_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}DmpWritingServer>)
ament_target_dependencies(${PROJECT_NAME}DmpWritingServer_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(pigpio REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test_hal_imu.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}I2cInit_test test/test_hal_imuI2cInit.cpp)
  ament_target_dependencies(${PROJECT_NAME}I2cInit_test ${dependencies} ${test_dependencies})
  target_link_libraries(${PROJECT_NAME}I2cInit_test ${PROJECT_NAME}I2cInit)

  ament_add_gtest(${PROJECT_NAME}DmpWritingServer_test test/test_hal_imuDmpWritingServer.cpp)
  ament_target_dependencies(${PROJECT_NAME}DmpWritingServer_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}DmpWritingServer_test ${PROJECT_NAME}DmpWritingServer)
endif()

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}DmpWritingServer_node
  ${PROJECT_NAME}I2cInit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}DmpWritingServer
  ${PROJECT_NAME}I2cInit
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_dependencies(${dependencies})
if(BUILD_TESTING)
  ament_export_dependencies(${test_dependencies})
endif()

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()