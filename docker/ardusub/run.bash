#!/bin/bash

xhost +local:docker

docker run -it \
  --privileged \
  --network=host \
  --env="ROS_DOMAIN_ID=27" \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="FASTRTPS_DEFAULT_PROFILES_FILE=/UnderWaterVehicle/docker/ardusub/rtps_udp_profile.xml" \
  --env="RMW_FASTRTPS_USE_QOS_FROM_XML=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev:/dev" \
  --volume="/run/user:/run/user" \
  --volume="${HOME}/UnderWaterVehicle:/UnderWaterVehicle" \
  --name="uuv_ardusub_dev" \
  hub.ci.dfl.ae/roboticslab/ros2_humble_x86_no_gpu:bluerov2
