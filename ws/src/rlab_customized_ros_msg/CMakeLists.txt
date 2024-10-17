cmake_minimum_required(VERSION 3.8)
project(rlab_customized_ros_msg)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StampedString.msg"
  "msg/RlabExample.msg"
  "msg/VoltaDiagnostic.msg"
  "msg/MurataDiag.msg"
  "msg/WiferionStatus.msg"
  "srv/ConvertENU.srv"
  "action/HardwareCheck.action"
  "action/Idle.action"
  "action/SCLock.action"
  "action/BarCode.action"
  "action/WallDocking.action"
  "action/WallUndocking.action"
  "action/Localization.action"
  "action/CrossRamp.action"
  "action/VLocalization.action"
  "action/TagDocking.action"
  "action/PreContact.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
