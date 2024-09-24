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
```
## Application
#### ROS High Level Control
#### Path Planning
#### Gstream + Deep Learning
