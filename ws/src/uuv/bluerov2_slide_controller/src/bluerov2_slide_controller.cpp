#include "bluerov2_slide_controller/bluerov2_slide_controller.h"

namespace bluerov2_slide_controller
{

void BlueRov2SlideController::init(){

  clock_ = this->get_clock();
  mavros_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 2);
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 2, std::bind(&BlueRov2SlideController::cmdCb, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "mavros/local_position/odom", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&BlueRov2SlideController::OdomCb, this, std::placeholders::_1));

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
  
  target_vy_ = 0.0;
  kp_ = 1.0;
  ki_ = 0.2;
  kd_ = 0.0;
  last_odom_ = clock_->now();
}

void BlueRov2SlideController::OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time current_t = clock_->now();
  rclcpp::Duration dt = current_t - last_odom_;

  error_ = cmd_vel_msg_.linear.y -  msg->twist.twist.linear.y;
  error_i_ += error_ * dt.seconds();
  error_d_ = error_/dt.seconds();

  compensate_vy_ = kp_* error_ + ki_*error_i_ + kd_*error_d_;
  if(compensate_vy_>0)
    compensate_vy_ = std::min(compensate_vy_, 0.8);
  else
    compensate_vy_ = std::max(compensate_vy_, -0.8);
  RCLCPP_INFO(this->get_logger(), "Target Y: %.2f, Compensate Y: %.3f, error_i: %.3f, error: %.2f, dt: %.5f", cmd_vel_msg_.linear.y , compensate_vy_, error_i_, error_, dt.seconds());
  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to get robot pose: %s", e.what());
  }

  tf2::Quaternion q_pose(transformStamped.transform.rotation.x, 
    transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  /*transform to RPY for weighting*/
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_pose).getRPY(roll, pitch, yaw);

  geometry_msgs::msg::TwistStamped m;
  m.header.frame_id = "map";
  m.twist.linear.x = cmd_vel_msg_.linear.x * cos(yaw) + (compensate_vy_) * cos(1.5707963 + yaw);
  m.twist.linear.y = cmd_vel_msg_.linear.x * sin(yaw) + (compensate_vy_) * sin(1.5707963 + yaw);
  m.twist.angular = cmd_vel_msg_.angular;
  mavros_cmd_pub_->publish(m);
  
  last_odom_ = current_t;
}

void BlueRov2SlideController::cmdCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_msg_ = (*msg);
}


}