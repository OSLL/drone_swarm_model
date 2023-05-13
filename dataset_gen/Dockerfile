FROM darcenrall/gazebo_simulator_base:20230216

ADD src /catkin_ws/src
ADD src/simulation/media /usr/share/gazebo-11/media

RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo ". /catkin_ws/devel/setup.bash " >> ~/.bashrc && \
    echo "python3 /catkin_ws/src/grader/performer.py; exit" >> ~/.bashrc #&& \    # TODO: uncomment to run grader
   # echo "python3 /catkin_ws/src/grader; exit" >> ~/.bashrc
