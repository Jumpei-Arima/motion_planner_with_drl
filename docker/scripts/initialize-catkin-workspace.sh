#!/bin/bash

# Source the updated ROS environment.
source /opt/ros/noetic/setup.bash

################################################################################

# Initialize and build the Catkin workspace.
cd /root/catkin_ws/ && catkin_make -DCMAKE_BUILD_TYPE=Release

# Source the Catkin workspace.
source /root/catkin_ws/devel/setup.bash
