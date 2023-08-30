#!/bin/sh

if [ -d "sysroot" ]; then rm -Rf sysroot; fi

mkdir workspace

cp -r ../src workspace
rm -r workspace/src/sim

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker build -t arm_ros2:humble -f ./Dockerfile/Dockerfile_ubuntu_arm .
docker run --name arm_sysroot arm_ros2:humble

docker container export -o sysroot.tar arm_sysroot
mkdir sysroot
tar -C sysroot -xf sysroot.tar lib usr opt etc
docker rm arm_sysroot

rm sysroot.tar
rm -r workspace