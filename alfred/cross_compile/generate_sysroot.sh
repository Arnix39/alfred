#!/bin/sh

if [ -d "sysroot" ]; then rm -Rf sysroot; fi

mkdir workspace

cp -r ../src workspace
rm -r workspace/src/sim

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker buildx create --use --name multi-arch-builder
docker buildx build --platform linux/arm64 -t arm_ros2:humble -f ./Dockerfile/Dockerfile_arm64_humble .
docker run --name alfred_cross arm_ros2:humble

docker container export -o sysroot.tar alfred_cross

mkdir sysroot
tar -C sysroot -xf sysroot.tar lib usr opt etc

docker rm alfred_cross

rm sysroot.tar
rm -r workspace