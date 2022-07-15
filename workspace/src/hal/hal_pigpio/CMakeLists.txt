cmake_minimum_required(VERSION 3.8)
project(hal_pigpio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)
find_package(hal_common REQUIRED)
find_package(pigpio REQUIRED)

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
  hal_common
  pigpio)

add_library(${PROJECT_NAME} OBJECT 
  src/hal_pigpio.cpp
  src/hal_pigpioI2c.cpp
  src/hal_pigpioInit.cpp
  src/hal_pigpioOutput.cpp
  src/hal_pigpioInput.cpp
  src/hal_pigpioImu.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node 
  src/hal_pigpio_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}>) 
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  #ament_lint_auto_find_test_dependencies()
 
  ament_add_gtest(${PROJECT_NAME}_init_test test/test_hal_pigpio_init.cpp)
  ament_target_dependencies(${PROJECT_NAME}_init_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_init_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_output_test test/test_hal_pigpio_output.cpp)
  ament_target_dependencies(${PROJECT_NAME}_output_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_output_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_input_test test/test_hal_pigpio_input.cpp)
  ament_target_dependencies(${PROJECT_NAME}_input_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_input_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_i2c_test test/test_hal_pigpio_i2c.cpp)
  ament_target_dependencies(${PROJECT_NAME}_i2c_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_i2c_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_imu_test test/test_hal_pigpio_imu.cpp)
  ament_target_dependencies(${PROJECT_NAME}_imu_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_imu_test ${PROJECT_NAME})
endif()

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()