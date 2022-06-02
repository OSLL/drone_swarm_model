FROM ros:noetic

SHELL ["/bin/bash", "-c"] 

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-gazebo-plugins ros-noetic-xacro \
    ros-noetic-rqt ros-noetic-rqt-common-plugins \
    ros-noetic-image-view \
    libignition-msgs-dev libignition-transport-dev \
    tmux

WORKDIR /catkin_ws

RUN . /opt/ros/noetic/setup.sh && \
    mkdir src && \
    catkin_init_workspace /catkin_ws/src 

ADD src /catkin_ws/src
ADD src/cottage/media /usr/share/gazebo-11/media

RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo ". /catkin_ws/devel/setup.bash " >> ~/.bashrc