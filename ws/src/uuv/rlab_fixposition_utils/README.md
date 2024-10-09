# This package use ECEF reference to convert the ENU coordinate

## Launch

```
ros2 launch enu_transform enu_transform_test_launch.py
```

## The reference point can be modified at:
 * enu_tranform.yaml file
 

### Argument (The coordinate is based on ECEF translation and Quaternion):
    * force_transformed_z_to_zero: true
    * ecef_ref_x: 3288216.3414
    * ecef_ref_y: 4745911.2612
    * ecef_ref_z: 2700930.4015
    * ecef_ref_quaternion_x: 0.501552
    * ecef_ref_quaternion_y: 0.238044
    * ecef_ref_quaternion_z: 0.740403
    * ecef_ref_quaternion_w: -0.378925

### data set:
    * under data folder, you will see 3 waypoint file on ECEF raw data record from fixposition


