#!/usr/bin/bash
export ROS_MASTER_URI=http://192.168.1.221:11311
export ROS_IP=192.168.1.211

exec "$@"
