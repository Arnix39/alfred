#!/bin/sh

. /opt/ros/humble/setup.sh
cd workspace
colcon build --build-base build/host --install-base install/host --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DSYNTHETIC_BUILD:BOOL=ON
colcon test --build-base build/host --install-base install/host
colcon test-result --test-result-base build/host
