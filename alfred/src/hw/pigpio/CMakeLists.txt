cmake_minimum_required(VERSION 3.8)
project(pigpio)

find_package(ament_cmake REQUIRED)
find_package(common_utils REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
  common_utils)

if(SYNTHETIC_BUILD)
  add_library(${PROJECT_NAME}_pi_mock OBJECT src/mock/raspberrypi_mock.cpp)
  ament_target_dependencies(${PROJECT_NAME}_pi_mock ${dependencies})

  add_library(${PROJECT_NAME} SHARED
    src/mock/pigpio_mock.cpp
    $<TARGET_OBJECTS:${PROJECT_NAME}_pi_mock>)
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})

else()
  add_library(${PROJECT_NAME}_command OBJECT src/command.c)
  target_compile_options(${PROJECT_NAME}_command PRIVATE -O3 -pthread -fpic)

  add_library(${PROJECT_NAME} SHARED
    src/pigpiod_if2.c
    $<TARGET_OBJECTS:${PROJECT_NAME}_command>)
endif()

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_copyright)
  ament_copyright(
    "include/mock"
    "src/mock"
  )
  find_package(ament_cmake_cpplint)
  ament_cpplint(
    "include/mock"
    "src/mock"
  )
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