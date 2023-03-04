#!/bin/sh

. /opt/ros/humble/setup.sh
cd workspace/cross_compile
./generate_sysroot.sh

cd ..
export TARGET_ARCH=aarch64
export CROSS_COMPILE=/usr/bin/aarch64-linux-gnu-
export SYSROOT=${WORKSPACE}/workspace/cross_compile/sysroot
export ROS2_INSTALL_PATH=${WORKSPACE}/workspace/install/target
export PYTHON_SOABI=cpython-310-aarch64-linux-gnu
export CC=/usr/bin/aarch64-linux-gnu-gcc
export CXX=/usr/bin/aarch64-linux-gnu-g++
export CROSS_COMPILE_DIR=${WORKSPACE}/workspace/cross_compile
colcon build --build-base build/target --install-base install/target --packages-skip-regex ros_gz --symlink-install --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON -DCMAKE_TOOLCHAIN_FILE=${CROSS_COMPILE_DIR}/cmake-toolchains/generic_linux.cmake --no-warn-unused-cli
