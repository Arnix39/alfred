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
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   hal_pigpioGetHandle.srv
   hal_pigpioSetInputMode.srv
   hal_pigpioSetOutputMode.srv
   hal_pigpioSetGpioHigh.srv
   hal_pigpioSetGpioLow.srv
   hal_pigpioSetPwmDutycycle.srv
   hal_pigpioSetPwmFrequency.srv
   hal_pigpioSendTriggerPulse.srv
   hal_pigpioI2cOpen.srv
   hal_pigpioI2cClose.srv
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

add_executable(${PROJECT_NAME}Init_node src/hal_pigpioInit.cpp)
add_dependencies(${PROJECT_NAME}Init_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}Init_node
  ${catkin_LIBRARIES}
  ${pigpio_LIBRAIRIES}
)

add_executable(${PROJECT_NAME}I2c_node src/hal_pigpioI2c.cpp)
add_dependencies(${PROJECT_NAME}I2c_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}I2c_node
  ${catkin_LIBRARIES}
  ${pigpio_LIBRAIRIES}
)

add_executable(${PROJECT_NAME}Output_node src/hal_pigpioOutput.cpp)
add_dependencies(${PROJECT_NAME}Output_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}Output_node
  ${catkin_LIBRARIES}
  ${pigpio_LIBRAIRIES}
)

add_executable(${PROJECT_NAME}Input_node src/hal_pigpioInput.cpp)
add_dependencies(${PROJECT_NAME}Input_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}Input_node
  ${catkin_LIBRARIES}
  ${pigpio_LIBRAIRIES}
)

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}Init_node
  ${PROJECT_NAME}I2c_node
  ${PROJECT_NAME}Output_node
  ${PROJECT_NAME}Input_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
