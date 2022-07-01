cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME hal_pigpio)
project(${PROJECT_NAME})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
find_package(pigpio MODULE REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
set(msg_files
  msg/HalPigpioEdgeChange.msg
  msg/HalPigpioHeartbeat.msg
  msg/HalPigpioAngles.msg
  msg/HalPigpioEncoderCount.msg
)

set(srv_files
  srv/HalPigpioGetHandle.srv
  srv/HalPigpioGetMode.srv
  srv/HalPigpioSetInputMode.srv
  srv/HalPigpioSetOutputMode.srv
  srv/HalPigpioSetPullUp.srv
  srv/HalPigpioSetPullDown.srv
  srv/HalPigpioClearResistor.srv
  srv/HalPigpioSetGpioHigh.srv
  srv/HalPigpioSetGpioLow.srv
  srv/HalPigpioSetPwmDutycycle.srv
  srv/HalPigpioSetPwmFrequency.srv
  srv/HalPigpioSendTriggerPulse.srv
  srv/HalPigpioI2cOpen.srv
  srv/HalPigpioI2cClose.srv
  srv/HalPigpioI2cReadByteData.srv
  srv/HalPigpioI2cReadWordData.srv
  srv/HalPigpioI2cReadBlockData.srv
  srv/HalPigpioI2cWriteByteData.srv
  srv/HalPigpioI2cWriteWordData.srv
  srv/HalPigpioI2cWriteBlockData.srv
  srv/HalPigpioReadGpio.srv
  srv/HalPigpioSetCallback.srv
  srv/HalPigpioI2cImuReading.srv
  srv/HalPigpioSetEncoderCallback.srv
  srv/HalPigpioSetMotorDirection.srv
)

#################################################
## Generate ROS messages, services and actions ##
#################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
 )

###########
## Build ##
###########
include_directories(
  include
  ../include
  ${catkin_INCLUDE_DIRS}
  ${pigpio_INCLUDE_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_init_lib OBJECT src/hal_pigpioInit.cpp)
ament_target_dependencies(${PROJECT_NAME}_init_lib rclcpp)
target_link_libraries(${PROJECT_NAME}_init_lib "${cpp_typesupport_target}") 

add_library(${PROJECT_NAME}_output_lib OBJECT src/hal_pigpioOutput.cpp)
ament_target_dependencies(${PROJECT_NAME}_output_lib rclcpp)
target_link_libraries(${PROJECT_NAME}_output_lib "${cpp_typesupport_target}") 

add_library(${PROJECT_NAME}_input_lib OBJECT src/hal_pigpioInput.cpp)
ament_target_dependencies(${PROJECT_NAME}_input_lib rclcpp)
target_link_libraries(${PROJECT_NAME}_input_lib "${cpp_typesupport_target}") 

add_library(${PROJECT_NAME}_i2c_lib OBJECT src/hal_pigpioI2c.cpp)
ament_target_dependencies(${PROJECT_NAME}_i2c_lib rclcpp)
target_link_libraries(${PROJECT_NAME}_i2c_lib "${cpp_typesupport_target}") 

add_library(${PROJECT_NAME}_imu_lib OBJECT src/hal_pigpioImu.cpp)
ament_target_dependencies(${PROJECT_NAME}_imu_lib rclcpp)
target_link_libraries(${PROJECT_NAME}_imu_lib "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_node 
  src/hal_pigpio.cpp 
  $<TARGET_OBJECTS:${PROJECT_NAME}_init_lib> 
  $<TARGET_OBJECTS:${PROJECT_NAME}_output_lib> 
  $<TARGET_OBJECTS:${PROJECT_NAME}_input_lib>
  $<TARGET_OBJECTS:${PROJECT_NAME}_i2c_lib>
  $<TARGET_OBJECTS:${PROJECT_NAME}_imu_lib> 
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)
target_link_libraries(${PROJECT_NAME}_node
  "${cpp_typesupport_target}"
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