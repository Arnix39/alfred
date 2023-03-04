#!/bin/sh

. /opt/ros/humble/setup.sh
cd workspace
colcon build --build-base build/host --install-base install/host --packages-skip-regex ros_gz --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DSYNTHETIC_BUILD:BOOL=ON --no-warn-unused-cli
colcon test --build-base build/host --install-base install/host --packages-skip-regex ros_gz
colcon test-result --test-result-base build/host
