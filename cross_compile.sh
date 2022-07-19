#!/bin/sh

export PROJECT_DIR=~/src/alfred

export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
export SYSROOT=$PROJECT_DIR/cross_compile/sysroot
export ROS2_INSTALL_PATH=$PROJECT_DIR/workspace/install
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE

colcon build --symlink-install \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
        -DCMAKE_TOOLCHAIN_FILE="$PROJECT_DIR/cross_compile/cmake-toolchains/generic_linux.cmake" \
        -DSECURITY=ON
