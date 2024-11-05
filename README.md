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
# BlueROV2 Simulation Setup Guide

### Clone the UnderWaterVehicle Repository:
```
git clone https://github.com/dfl-rlab/UnderWaterVehicle.git
```
## Docker Setup

- ### **Build Docker Image for ArduPilot BlueROV2**  
   Navigate to the Docker directory and build the image:           
```
cd UnderWaterVehicle/docker
docker build -t hub.ci.dfl.ae/roboticslab/ros2_humble_x86_no_gpu:bluerov2 -f Dockerfile_ardupilot_bluerov2 .
```
- ### **Start the Container Environment**         
   Use Docker Compose to launch the environment:
```
cd UnderWaterVehicle/docker
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
docker exec -it uuv_ardusub_dev bash
cd UnderWaterVehicle/ws/
source /opt/ros/humble/setup.bash
source install/setup.bash 
rviz2
```

## Running the Autonomous ROV Controller and Client
- ### **Run the Autonomous ROV Controller**
   Start the controller to handle navigation and movement commands:
```
docker exec -it uuv_ardusub_dev bash
cd UnderWaterVehicle/ws/
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

