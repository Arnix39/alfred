#!/bin/sh

if ! [ -d "../install" ]; then mkdir ../install; fi
if [ -d "../install/target" ]; then rm -r ../install/target; fi

mkdir ../install/target
mkdir workspace

cp -r ../src workspace
rm -r workspace/src/sim

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker build -t arm_ros2:humble -f ./Dockerfile/Dockerfile_arm64_humble .
docker run --name alfred_cross arm_ros2:humble

docker cp alfred_cross:/ros2_ws/log/. ../log
docker cp alfred_cross:/ros2_ws/install.tar.gz .

tar -C ../install/target --strip-components 1 -xf install.tar.gz

rm install.tar.gz
rm -r workspace
