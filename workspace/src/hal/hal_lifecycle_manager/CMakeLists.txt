cmake_minimum_required(VERSION 3.8)
project(hal_lifecycle_manager)

find_package(ament_cmake REQUIRED)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()