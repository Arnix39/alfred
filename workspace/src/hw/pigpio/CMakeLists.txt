cmake_minimum_required(VERSION 3.8)
project(pigpio)

find_package(ament_cmake REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

if(SYNTHETIC_BUILD)
  add_library(${PROJECT_NAME}_pi_mock OBJECT src/mock/raspberrypi_mock.cpp)

  add_library(${PROJECT_NAME} SHARED 
    src/mock/pigpio_mock.cpp
    $<TARGET_OBJECTS:${PROJECT_NAME}_pi_mock>)
else()
  add_library(${PROJECT_NAME}_command OBJECT src/command.c)
  target_compile_options(${PROJECT_NAME}_command PRIVATE -O3 -pthread -fpic)

  add_library(${PROJECT_NAME} SHARED 
    src/pigpiod_if2.c 
    $<TARGET_OBJECTS:${PROJECT_NAME}_command>)
endif()

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

ament_package()