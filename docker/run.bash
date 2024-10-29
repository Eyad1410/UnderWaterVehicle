#!/bin/bash

xhost +local:docker

docker run -it\
  --privileged \
  --network=host \
  --env="ROS_DOMAIN_ID=27" \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev:/dev" \
  --volume="/run/user:/run/user" \
  --volume="${HOME}/UnderWaterVehicle:/UnderWaterVehicle" \
  --name="bluerov2-dev" \
  hub.ci.dfl.ae/roboticslab/ros2_humble_x86_no_gpu:bluerov2
