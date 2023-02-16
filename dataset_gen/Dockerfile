FROM darcenrall/gazebo_simulator_base:20230216

ADD src /catkin_ws/src
ADD src/cottage/media /usr/share/gazebo-11/media

RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo ". /catkin_ws/devel/setup.bash " >> ~/.bashrc