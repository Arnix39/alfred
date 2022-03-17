cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME hal_pigpio)
project(${PROJECT_NAME})

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
find_package(pigpio MODULE REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   hal_pigpioEdgeChangeMsg.msg
   hal_pigpioHeartbeatMsg.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   hal_pigpioGetHandle.srv
   hal_pigpioGetMode.srv
   hal_pigpioSetInputMode.srv
   hal_pigpioSetOutputMode.srv
   hal_pigpioSetPullUp.srv
   hal_pigpioSetPullDown.srv
   hal_pigpioClearResistor.srv
   hal_pigpioSetGpioHigh.srv
   hal_pigpioSetGpioLow.srv
   hal_pigpioSetPwmDutycycle.srv
   hal_pigpioSetPwmFrequency.srv
   hal_pigpioSendTriggerPulse.srv
   hal_pigpioI2cOpen.srv
   hal_pigpioI2cClose.srv
   hal_pigpioI2cReadByteData.srv
   hal_pigpioI2cReadWordData.srv
   hal_pigpioI2cReadBlockData.srv
   hal_pigpioI2cWriteByteData.srv
   hal_pigpioI2cWriteWordData.srv
   hal_pigpioI2cWriteBlockData.srv
   hal_pigpioReadGpio.srv
   hal_pigpioSetCallback.srv
 )

## Generate added messages and services with any dependencies listed here
  generate_messages(
   DEPENDENCIES 
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp 
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pigpio_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_init_lib OBJECT src/hal_pigpioInit.cpp)
add_dependencies(${PROJECT_NAME}_init_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_output_lib OBJECT src/hal_pigpioOutput.cpp)
add_dependencies(${PROJECT_NAME}_output_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_input_lib OBJECT src/hal_pigpioInput.cpp)
add_dependencies(${PROJECT_NAME}_input_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_i2c_lib OBJECT src/hal_pigpioI2c.cpp)
add_dependencies(${PROJECT_NAME}_i2c_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_imu_lib OBJECT src/hal_pigpioImu.cpp)
add_dependencies(${PROJECT_NAME}_imu_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node 
  src/hal_pigpio.cpp 
  $<TARGET_OBJECTS:${PROJECT_NAME}_init_lib> 
  $<TARGET_OBJECTS:${PROJECT_NAME}_output_lib> 
  $<TARGET_OBJECTS:${PROJECT_NAME}_input_lib>
  $<TARGET_OBJECTS:${PROJECT_NAME}_i2c_lib>
  $<TARGET_OBJECTS:${PROJECT_NAME}_imu_lib> 
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${pigpio_LIBRAIRIES}
)

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
