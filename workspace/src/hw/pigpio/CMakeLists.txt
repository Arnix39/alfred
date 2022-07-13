cmake_minimum_required(VERSION 3.8)
project(pigpio)

find_package(ament_cmake REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
)

if(BUILD_TESTING)
  add_library(${PROJECT_NAME} SHARED src/pigpio_mock.cpp)
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})
else()
  add_library(${PROJECT_NAME}_command SHARED src/command.c)
  ament_target_dependencies(${PROJECT_NAME}_command ${dependencies})

  add_library(${PROJECT_NAME} SHARED src/pigpiod_if2.c)
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})
  target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_command)
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
ament_export_dependencies(${dependencies})

ament_package()