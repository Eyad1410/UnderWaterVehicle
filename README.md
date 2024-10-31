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
## How to run Mavros 
```
./run.bash
cd UnderWaterVehicle/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash 
ros2 launch mavros apm.launch
```
## How to run Rviz2 
To launch the local_odom2tf node, run:
```
ros2 run mavros_local_odom_tf local_odom2tf 
```
```
docker exec -it uuv_ardusub_dev bash
cd UnderWaterVehicle/ws/
source install/setup.bash 
rviz2
```
## Ardusub_Simulator in Host
```
cd UnderWaterVehicle/docker/
docker start uuv_ardusub_simulator
docker start uuv_ardusub_simulator_apm
```
## How to run Action Server and Client
```
docker exec -it uuv_ardusub_dev bash
cd UnderWaterVehicle/ws/
colcon build --packages-select rlab_customized_ros_msg control
source install/setup.bash
ros2 run control autonomous_rov_server
```
## Open a new terminal to run Client in uuv_ardusub_dev container 
```
docker exec -it uuv_ardusub_dev bash
cd UnderWaterVehicle/ws/
source install/setup.bash
ros2 run control autonomous_rov_client
```
<p align='center'>
    <img src="https://github.com/dfl-rlab/documentation_materials/blob/master/uuv/uuv_snail.png" width="780" height="560"/>
</p>

## Building the Workspace
```
colcon build --symlink-install
```
## Application
#### ROS High Level Control
#### Path Planning
#### Gstream + Deep Learning
