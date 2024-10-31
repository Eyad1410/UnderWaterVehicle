# control_as 
This package provides an autonomous control system for a remotely operated vehicle (ROV). The control system includes an action server and client (autonomous_rov_controller.py and autonomous_rov_client.py), which enable the ROV to perform automated snail pattern.

#### Dependence requirement:
  - rlab_customized_ros_msg


#### Quickly testing with two terminal (terminal1 action server, terminal2 action client) -> there are two methods available to trigger the action server: using a ROS command or a Python script.

 Terminal1 :

```
ros2 run control_as autonomous_rov_controller.py
```

 Terminal2 (ros command) :

```
ros2 action send_goal /snail_pattern rlab_customized_ros_msg/action/SnailPattern "{initial_side_length: 2.0, increment: 2.0, max_side_length: 10.0}"

```

Terminal2  (python script):
```
python3 autonomous_rov_client.py
```
