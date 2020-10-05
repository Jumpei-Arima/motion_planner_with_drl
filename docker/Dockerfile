FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y --no-install-recommends \
                   git \
                   python3-pip \
                   ros-noetic-tf* \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN mkdir -p catkin_ws/src && \
    cd catkin_ws && \
    source /opt/ros/noetic/setup.bash; catkin_make

RUN echo source /root/catkin_ws/devel/setup.bash >> ~/.bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH
ENV ROS_WORKSPACE=/root/catkin_ws

RUN pip3 install -U pip

RUN pip3 install torch pfrl

WORKDIR /root

COPY ./ros_entrypoint.sh /

CMD ["/ros_entrypoint.sh"]

WORKDIR /root/catkin_ws
