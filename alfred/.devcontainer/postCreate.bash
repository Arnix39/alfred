#!/bin/bash

rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble

echo ". /opt/ros/humble/setup.bash" | sudo tee -a ~/.bashrc > /dev/null