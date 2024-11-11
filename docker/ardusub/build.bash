#!/bin/bash

docker build -t hub.ci.dfl.ae/roboticslab/ros2_humble_x86_no_gpu:bluerov2 -f Dockerfile_ardupilot_bluerov2 --build-arg SSH_PRIVATE_KEY="`cat ~/.ssh/id_rsa`" .
