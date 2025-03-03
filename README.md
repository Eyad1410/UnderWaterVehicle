# UnderWaterVehicle
UUV
## Git
```
cd UnderWaterVehicle
git status
git add the file
git commit -m "add a new node"
git push
```
## Hardware and Software Setup

#### MavROS - Humble
BlueOS by default running MAVROS with ROS Noetic.
- [ ] TODO: Turn off MAVROS1

With MAVROS - Humble, we need:
```
<arg name="fcu_url" default="tcp://192.168.2.2:5777" />
```
- [ ] TODO: Dockerfile of MAVROS2

#### GStream
By default, BlueROV2 runs Gstream udp
- [ ] TODO: Compile [gstream-ros2](https://github.com/BrettRD/ros-gst-bridge) and visualize streaming in RViz2
```
GST_PLUGIN_PATH=install/gst_bridge/lib/gst_bridge gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ udpsrc port=5600 ! 'application/x-rtp,encoding-name=H264,payload=96,clock-rate=90000' ! rtph264depay ! avdec_h264 ! queue leaky=1 ! decodebin ! videoconvert ! rosimagesink ros-topic="/fixposition/image" sync=false
```

# Docker Image Qground_control 

### Clone the UnderWaterVehicle Repository
```
git clone https://github.com/dfl-rlab/UnderWaterVehicle.git
```
- ### **To build the Docker image for QGroundControl**
   navigate to /docker/qground directory and build the image: 
```
cd UnderWaterVehicle/docker/qground
docker build -t qgc_ubuntu_no_gpu -f Dockerfile_qground_bluerov2 .
```
- ### **run the container, using this command:**
```
./run.bash
 ```
# Docker Image BlueROV2 Gazebo (`bluerov2_gz`)

### Clone the UnderWaterVehicle Repository
```
git clone https://github.com/dfl-rlab/UnderWaterVehicle.git
```

This repository contains all the necessary files to build and run the `bluerov2_gz` simulation environment.

- ### **To build the Docker image for BlueROV2 Gazebo**
   Navigate to `/docker/bluerov2_gz/` directory and build the image:
```
cd UnderWaterVehicle/docker/bluerov2_gz/
./build.bash

//docker build -t hub.ci.dfl.ae/roboticslab/bluerov2_gz:latest \
  -f Dockerfile_bluerov2_gz .
```

**Docker image `bluerov2_gz`**

- ### **Run the container, using:**
```
./run.bash
```

**To launch `bluerov2_gz` container**

- ### **Build the ROS 2 Workspace**
   After the container is running, navigate to the ROS 2 workspace:
```
cd /UnderWaterVehicle/bluerov2_gz/ros2_ws
```
   Build the **Gazebo launch package** and **source the ROS 2 workspace**:
```
colcon build --packages-select gazebo_launch
source install/setup.bash
```

- ### **Launch Gazebo with BlueROV2**
```
ros2 launch gazebo_launch gazebo_launch.py
```

# BlueROV2 Simulation Setup Guide

### Clone the UnderWaterVehicle Repository:
```
git clone https://github.com/dfl-rlab/UnderWaterVehicle.git
```
## Docker Setup

- ### **Build Docker Image for ArduPilot BlueROV2**  
   Navigate to the Docker directory and build the image:           
```
cd UnderWaterVehicle/docker/ardusub
./build.bash
//docker build -t hub.ci.dfl.ae/roboticslab/ros2_humble_x86_no_gpu:bluerov2 -f Dockerfile_ardupilot_bluerov2 .
```
- ### **Start the Container Environment**         
   Use Docker Compose to launch the environment:
```
cd UnderWaterVehicle/docker/ardusub
docker compose -f uuv-dev.yaml up
```

## Build and Setup the ROS 2 Workspace
- ### **Access the Docker container and set up the ROS 2 workspace:**
   Run the below commands to access container, set up the environment, and build the workspace:
```
docker exec -it uuv_ardusub_dev bash
cd ~/UnderWaterVehicle/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash 
```

## Running Rviz2 for Visualization
- ### **Launch the local_odom2tf Node:**
```
ros2 run mavros_local_odom_tf local_odom2tf 
```
- ### **Run Rviz2:**
```
docker start uuv_ardusub_dev
docker exec -it uuv_ardusub_dev bash
cd /UnderWaterVehicle/ws/
source /opt/ros/humble/setup.bash
source install/setup.bash 
rviz2
```

## Running the Autonomous ROV Controller and Client
- ### **Run the Autonomous ROV Controller**
   Start the controller to handle navigation and movement commands:
```
docker exec -it uuv_ardusub_dev bash
cd /UnderWaterVehicle/ws
source install/setup.bash
ros2 run control_as autonomous_rov_controller.py
```
- ### **Run the Autonomous ROV Client**
   Open a separate terminal for the client to send actions to the controller:
```
docker exec -it uuv_ardusub_dev bash
cd UnderWaterVehicle/ws/
source install/setup.bash
ros2 run control_as autonomous_rov_client.py
```

## Testing Action Client Goals
- ### **Snail Pattern Movement**
   To test movements like the Snail Pattern, use this action goal:
```
ros2 action send_goal /snail_pattern rlab_customized_ros_msg/action/SnailPattern \
"{initial_side_length: 2.0, increment: 2.0, max_side_length: 20.0}"
```
--------------------------------------------------------------------
# Snail Pattern Trajectories at Different Speeds for BlueROV2
<table align='center'>
  <tr width="100%">
    <td width="50%"><img src="https://github.com/dfl-rlab/documentation_materials/blob/master/uuv/uuv_snail_1.0_mps.png" width="400" height="260"/><p align='center'>uuv_snail pattern 1.0 m/s</p></td>
    <td width="50%"><img src="https://github.com/dfl-rlab/documentation_materials/blob/master/uuv/uuv_snail_1.5_mps.png" width="400" height="260"/><p align='center'>uuv_snail pattern 1.5 m/s</p></td>
  </tr>
  <tr width="100%">
    <td width="50%"><img src="https://github.com/dfl-rlab/documentation_materials/blob/master/uuv/uuv_snail_2.0_mps.png" width="400" height="260"/><p align='center'>uuv_snail pattern 2.0 m/s</p></td>
    <td width="50%"><img src="https://github.com/dfl-rlab/documentation_materials/blob/master/uuv/uuv_snail_2.0_mps_v2.png" width="400" height="260"/><p align='center'>uuv_snail pattern 2.0 m/s 2nd attempt</p></td>
  </tr>
</table> 

## Application
#### ROS High Level Control
#### Path Planning
#### Gstream + Deep Learning

