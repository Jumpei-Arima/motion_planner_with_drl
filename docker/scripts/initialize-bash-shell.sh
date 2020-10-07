#!/bin/bash

################################################################################

# Link the default shell 'sh' to Bash.
alias sh='/bin/bash'

################################################################################

# Configure the terminal.

# Disable flow control. If enabled, inputting 'ctrl+s' locks the terminal until inputting 'ctrl+q'.
stty -ixon

################################################################################

# Configure 'umask' for giving read/write/execute permission to group members.
umask 0002

################################################################################

# Source the ROS environment.
echo "Sourcing the ROS environment from '/opt/ros/noetic/setup.bash'."
source /opt/ros/noetic/setup.bash

# Source the Catkin workspace.
echo "Sourcing the Catkin workspace from '/root/catkin_ws/devel/setup.bash'."
source /root/catkin_ws/devel/setup.bash

################################################################################

# Add the Catkin workspace to the 'ROS_PACKAGE_PATH'.
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/catkin_ws/src
export ROS_WORKSPACE=/root/catkin_ws

################################################################################

# Set ROS network interface.
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=`hostname -I | cut -d' ' -f1`
echo "ROS_IP is set to '${ROS_IP}'."
export ROS_HOME=~/.ros

################################################################################

function cmk(){
    source /root/docker/scripts/initialize-catkin-workspace.sh
}

################################################################################

# Move to the working directory.
cd /root/catkin_ws/src/mpdrl_ros
