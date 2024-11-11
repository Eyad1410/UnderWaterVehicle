#!/bin/bash

# Build the Docker image for QGroundControl
docker build -t hub.ci.dfl.ae/roboticslab/qgc_ubuntu_no_gpu:latest \
  -f Dockerfile_qground_bluerov2 .

