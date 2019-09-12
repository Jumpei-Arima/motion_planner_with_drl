#!/bin/bash

echo "=== run_docker ==="

xhost +local:docker

docker run -it --rm \
  --env=QT_X11_NO_MITSHM=1 \
  --env=DISPLAY=$DISPLAY \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${PWD}:/root/catkin_ws/src/motion_planner_with_drl" \
  --net='host' \
  --name="ros_mpdrl" \
  arijun/ros_mpdrl \
  bash

