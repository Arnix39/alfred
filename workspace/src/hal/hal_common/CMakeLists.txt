cmake_minimum_required(VERSION 3.8)
project(hal_common)

find_package(ament_cmake REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

add_library(${PROJECT_NAME} INTERFACE)

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