cmake_minimum_required(VERSION 3.8)
project(hal_pigpio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hal_pigpio_interfaces REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
find_package(pigpio MODULE REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ../include
  ${pigpio_INCLUDE_DIRS}
)

set(dependencies
rclcpp
hal_pigpio_interfaces
)

add_library(${PROJECT_NAME}_init OBJECT src/hal_pigpioInit.cpp)
ament_target_dependencies(${PROJECT_NAME}_init ${dependencies})

add_library(${PROJECT_NAME}_output OBJECT src/hal_pigpioOutput.cpp)
ament_target_dependencies(${PROJECT_NAME}_output ${dependencies})

add_library(${PROJECT_NAME}_input OBJECT src/hal_pigpioInput.cpp)
ament_target_dependencies(${PROJECT_NAME}_input ${dependencies})

add_library(${PROJECT_NAME}_i2c OBJECT src/hal_pigpioI2c.cpp)
ament_target_dependencies(${PROJECT_NAME}_i2c ${dependencies})

add_library(${PROJECT_NAME}_imu OBJECT src/hal_pigpioImu.cpp)
ament_target_dependencies(${PROJECT_NAME}_imu ${dependencies})

add_executable(${PROJECT_NAME}_node 
  src/hal_pigpio.cpp 
  $<TARGET_OBJECTS:${PROJECT_NAME}_init> 
  $<TARGET_OBJECTS:${PROJECT_NAME}_output> 
  $<TARGET_OBJECTS:${PROJECT_NAME}_input>
  $<TARGET_OBJECTS:${PROJECT_NAME}_i2c>
  $<TARGET_OBJECTS:${PROJECT_NAME}_imu> 
)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node
  ${pigpio_LIBRAIRIES}
  ${rclcpp_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()