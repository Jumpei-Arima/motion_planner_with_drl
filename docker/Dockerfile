FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget \
        curl \
        vim \
        git \
        python3-pip \
        ros-noetic-tf*

WORKDIR /root

RUN mkdir -p catkin_ws/src && \
    cd catkin_ws && \
    source /opt/ros/noetic/setup.bash; catkin_make

RUN pip3 install -U pip
RUN pip3 install torch pfrl

# Copy the helper scripts to the working directory.
COPY ./docker/scripts/. /root/docker/scripts/
RUN chmod -R +x /root/docker/scripts/*

# Setup the Bash shell environment with '/root/.bashrc'.
RUN echo "source /root/docker/scripts/initialize-bash-shell.sh" >> /root/.bashrc

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root
ENTRYPOINT []
CMD ["/bin/bash"]
