FROM darcenrall/gazebo_simulator_base:20230216

RUN apt update && apt install --no-install-recommends -y \
    python3-pip

ADD src /catkin_ws/src
ADD src/simulation/media /usr/share/gazebo-11/media
ADD requirements.txt requirements.txt

RUN . /opt/ros/noetic/setup.sh && \
    pip install -r requirements.txt && \
    catkin_make

RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo ". /catkin_ws/devel/setup.bash " >> ~/.bashrc #&& \    # TODO: uncomment to run grader
   # echo "python3 /catkin_ws/src/grader; exit" >> ~/.bashrc
