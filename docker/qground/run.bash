#!/bin/bash

# Allow Docker to access the display
xhost +local:docker

# Run the Docker container
docker run -it \
  --privileged \
  --network=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="FASTRTPS_DEFAULT_PROFILES_FILE=/home/bluerov2/rtps_udp_profile.xml" \
  --env="RMW_FASTRTPS_USE_QOS_FROM_XML=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev:/dev" \
  --volume="/run/user:/run/user" \
  --volume="${HOME}/UnderWaterVehicle:/UnderWaterVehicle" \
  --name="qgc_bluerov2_dev" \
  qgc_ubuntu_no_gpu

