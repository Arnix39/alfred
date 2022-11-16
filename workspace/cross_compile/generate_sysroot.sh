#!/bin/sh

if [ -d "sysroot" ]; then rm -Rf sysroot; fi

mkdir qemu-user-static
mkdir workspace

cp /usr/bin/qemu-*-static qemu-user-static
cp -r ../src workspace

docker build -t arm_ros2:latest -f ./Dockerfile/Dockerfile_ubuntu_arm .
docker run --name arm_sysroot arm_ros2:latest

docker container export -o sysroot.tar arm_sysroot
mkdir sysroot
tar -C sysroot -xf sysroot.tar lib usr opt etc
docker rm arm_sysroot

rm sysroot.tar
rm -r qemu-user-static
rm -r workspace