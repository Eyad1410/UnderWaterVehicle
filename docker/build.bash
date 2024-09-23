#!/bin/bash

docker build -t hub.ci.dfl.ae/roboticslab/ros2_humble_x86_no_gpu:mavros --build-arg SSH_PRIVATE_KEY="`cat ~/.ssh/id_rsa`" . -f Dockerfile_ros2_humble_x86_no_gpu_dev_mavros
