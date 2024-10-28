#include "bluerov2_slide_controller/bluerov2_slide_controller.h"

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<bluerov2_slide_controller::BlueRov2SlideController>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}