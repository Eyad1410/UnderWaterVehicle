cmake_minimum_required(VERSION 3.5)
project(bluerov2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  mavros_msgs
)


include_directories(
  include
)

# Add your C++ executables
add_executable(arm_and_move_client
  src/ArmServiceClient.cpp
  src/main.cpp
)

ament_target_dependencies(arm_and_move_client
  ${dependencies}
)

# Include directories for header files
target_include_directories(arm_and_move_client
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against ROS 2 libraries
target_link_libraries(arm_and_move_client
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
)

# Install executables
install(TARGETS
  arm_and_move_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

