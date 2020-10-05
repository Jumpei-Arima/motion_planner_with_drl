#!/bin/bash

set -e

ldconfig

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

exec "$@"
