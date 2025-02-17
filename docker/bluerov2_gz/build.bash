#!/usr/bin/env bash

# Build the Docker image for BlueROV2 Gazebo
docker build -t hub.ci.dfl.ae/roboticslab/bluerov2:gz \
  -f Dockerfile_bluerov2_gz .
