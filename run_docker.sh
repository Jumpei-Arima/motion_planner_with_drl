#!/bin/bash

echo "=== run_docker ==="

docker run -it --rm \
  --volume="${PWD}:/root/catkin_ws/src/mpdrl_ros" \
  --net='host' \
  --name="mpdrl" \
  arijun/mpdrl \
  bash -c "catkin_make && roslaunch mpdrl_ros local_planner.launch"
