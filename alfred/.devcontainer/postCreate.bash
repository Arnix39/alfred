#!/bin/bash

sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
sudo chown -R arnix /home/workspace/

# Quick and dirty fix => to be investigated further
sudo cp /home/workspace/cross_compile/sysroot/usr/lib/aarch64-linux-gnu/libpython3.10.so /usr/lib/aarch64-linux-gnu/libpython3.10.so

echo ". /opt/ros/humble/setup.bash" | sudo tee -a ~/.bashrc > /dev/null