cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME sim)
project(${PROJECT_NAME})

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

#############
## Install ##
#############
install(DIRECTORY meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
