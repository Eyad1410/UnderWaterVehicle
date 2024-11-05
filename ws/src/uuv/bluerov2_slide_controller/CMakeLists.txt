cmake_minimum_required(VERSION 3.8)
project(bluerov2_slide_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
)

add_library(bluerov2_slide_controller_lib SHARED
  src/bluerov2_slide_controller.cpp
)
ament_target_dependencies(bluerov2_slide_controller_lib
  ${dependencies}
)


add_executable(bluerov2_slide_controller
  src/main.cpp
)

target_link_libraries(bluerov2_slide_controller bluerov2_slide_controller_lib)

ament_target_dependencies(bluerov2_slide_controller
  ${dependencies}
)

install(TARGETS
  bluerov2_slide_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
