cmake_minimum_required(VERSION 3.8)
project(app_lifecycle_manager)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()