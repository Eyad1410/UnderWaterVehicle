#!/usr/bin/env bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Specific for NVIDIA drivers, required for OpenGL >= 3.3
docker run -it \
    --name bluerov2_gz \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "/run/user:/run/user" \
    -v "${HOME}/UnderWaterVehicle:/UnderWaterVehicle" \
    --privileged \
    --security-opt seccomp=unconfined \
    --gpus all \
    hub.ci.dfl.ae/roboticslab/bluerov2_gz:latest



