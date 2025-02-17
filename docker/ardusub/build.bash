#!/bin/bash

docker build -t hub.ci.dfl.ae/roboticslab/bluerov2:apm -f Dockerfile_ardupilot_bluerov2 --build-arg SSH_PRIVATE_KEY="`cat ~/.ssh/id_rsa`" .
