#!/bin/bash

echo "=== run_docker ==="

docker run -it --rm \
  --volume="${PWD}/../:/root/catkin_ws/src/mpdrl_ros" \
  --net='host' \
  --name="mpdrl_ros" \
  arijun/mpdrl_ros \
  bash -c "catkin_make && export ROS_MASTER_URI=http://192.168.2.150:11311 && export ROS_IP=192.168.2.101 && roslaunch mpdrl_ros navigation.launch"
