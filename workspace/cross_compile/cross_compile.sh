#!/bin/sh

if ! [ -d "../build" ]; then mkdir ../build; fi
if ! [ -d "../install" ]; then mkdir ../install; fi

mkdir workspace

cp -r ../src workspace
rm -r workspace/src/sim

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker build -t arm_ros2:humble -f ./Dockerfile/Dockerfile_arm64_humble .
docker run --name alfred_cross arm_ros2:humble

docker cp alfred_cross:/ros2_ws/build/. ../build/target
docker cp alfred_cross:/ros2_ws/install/. ../install/target
docker cp alfred_cross:/ros2_ws/log/. ../log

rm -r workspace
