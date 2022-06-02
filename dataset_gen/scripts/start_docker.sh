#!/bin/bash

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="$(pwd)/src:/catkin_ws/src:rw" \
    --device=/dev/dri:/dev/dri \
    --name=dataset_gen \
    dataset_gen_image  \
    bash
