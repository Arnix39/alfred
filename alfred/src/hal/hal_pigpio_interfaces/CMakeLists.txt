cmake_minimum_required(VERSION 3.8)
project(hal_pigpio_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
set(msg_files
  msg/HalPigpioEdgeChange.msg
  msg/HalPigpioEncoderCount.msg
)

set(srv_files
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
)

#################################################
## Generate ROS messages, services and actions ##
#################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()