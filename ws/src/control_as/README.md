#  Util AS
Util AS (example as_idle.py is python action server  , include prevent double trigger cause multi thread issue and action client cancel request)

#### Dependence requirement:
  - rlab_customized_ros_msg


#### Quickly testing with two terminal (terminal1 action server, terminal2 action client) -> there are two methods available to trigger the action server: using a ROS command or a Python script.

 Terminal1 :

```
ros2 run util_AS as_idle.py 
```

 Terminal2 (ros command) :

```
ros2 action send_goal /as_idle_test rlab_customized_ros_msg/action/Idle {robot_id:\ \'\',idle_time:\ 10.0\}
```

Terminal2  (python script):
```
python3 example_trigger_client.py
```
