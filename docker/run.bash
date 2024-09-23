#!/bin/bash

xhost +local:docker

docker run -it --rm\
  --privileged \
  --network=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev:/dev" \
  --volume="${HOME}/UnderWaterVehicle:/UnderWaterVehicle" \
  --name="mavros" \
  hub.ci.dfl.ae/roboticslab/ros2_humble_x86_no_gpu:mavros



