FROM ros:kinetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y sudo\
                       wget\
                       lsb-release\
                       mesa-utils \
                       gfortran \
                       vim \
                       python-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install typing

RUN apt-get update

RUN apt-get install -y ros-kinetic-pcl-ros* \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN mkdir -p catkin_ws/src

RUN cd catkin_ws/src && source /opt/ros/kinetic/setup.bash; catkin_init_workspace

RUN cd catkin_ws && source /opt/ros/kinetic/setup.bash; catkin_make

RUN cd /root && echo source /root/catkin_ws/devel/setup.bash >> .bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

RUN git clone https://github.com/pytorch/pytorch \

WORKDIR /root/pytorch
RUN git submodule update --init --recursive \
    && python setup.py build\
    && mkdir build_libtorch \
    && cd build_libtorch \
    && python ../tools/build_libtorch.py

WORKDIR /root

COPY ./ros_entrypoint.sh /

CMD ["/ros_entrypoint.sh"]
