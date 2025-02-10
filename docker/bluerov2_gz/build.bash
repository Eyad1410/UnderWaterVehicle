#!/usr/bin/env bash

# Build the Docker image for BlueROV2 Gazebo
docker build -t hub.ci.dfl.ae/roboticslab/bluerov2_gz:latest \
  -f Dockerfile_bluerov2_gz .
