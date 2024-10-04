#!/bin/bash

sudo rosdep update
sudo apt update
sudo rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble

echo ". /opt/ros/humble/setup.bash" | sudo tee -a ~/.bashrc > /dev/null