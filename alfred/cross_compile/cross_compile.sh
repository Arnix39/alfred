#!/bin/sh

sudo apt-get update

rosdep update

rosdep install -y \
    --rosdistro humble \
    --from-paths src \
    --ignore-src
       
. /opt/ros/humble/setup.bash
colcon build --symlink-install \
             --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
             --no-warn-unused-cli