#!/bin/bash

echo "=== run_docker ==="

if [ $# -eq 0 ];
then
    cmd="bash"
else
    cmd="catkin_make && "$@
    cmd="bash -c "${cmd}
fi

echo ${cmd}

docker run -it --rm \
  --volume="${PWD}:/root/catkin_ws/src/mpdrl_ros" \
  --net='host' \
  --name="mpdrl_ros" \
  arijun/mpdrl_ros \
  ${cmd}
