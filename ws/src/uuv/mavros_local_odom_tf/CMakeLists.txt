cmake_minimum_required(VERSION 3.8)
project(mavros_local_odom_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)


include_directories(
  include
)

set(dependencies
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

add_executable(local_odom2tf src/local_odom2tf.cpp)
ament_target_dependencies(local_odom2tf 
  ${dependencies}
)

add_executable(cmd2tf_map2baselink_node src/cmd2tf_map2baselink.cpp)
ament_target_dependencies(cmd2tf_map2baselink_node 
  ${dependencies}
)


install(TARGETS
  local_odom2tf
  cmd2tf_map2baselink_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/enu_tf_verify.py
  scripts/fp_ecef_plot.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch data DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
