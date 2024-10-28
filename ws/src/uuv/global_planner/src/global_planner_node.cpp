#include <global_planner/global_planner.h>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto perception_3d = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_global");
  auto global_planner_node = std::make_shared<global_planner::Global_Planner>("global_planner");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(perception_3d);
  perception_3d->initial();
  executor.add_node(global_planner_node);
  global_planner_node->initial(perception_3d);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}