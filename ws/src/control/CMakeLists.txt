cmake_minimum_required(VERSION 3.5)
project(control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify the action file to be processed
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
)

find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

ament_package()

