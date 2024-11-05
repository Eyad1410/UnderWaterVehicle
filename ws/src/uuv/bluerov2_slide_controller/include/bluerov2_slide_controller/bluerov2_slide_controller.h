#ifndef ROBOT_SAFETY_WATCHDOG_HPP_
#define ROBOT_SAFETY_WATCHDOG_HPP_

#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
#include "rclcpp/rclcpp.hpp"
 
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

/*TF listener, although it is included in sensor.h*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace bluerov2_slide_controller
{
  class BlueRov2SlideController: public rclcpp::Node{

    public:

      BlueRov2SlideController():Node("bluerov2_slide_controller"){init();};

    private:

      rclcpp::Clock::SharedPtr clock_;

      std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tfl_;
      rclcpp::CallbackGroup::SharedPtr tf_listener_group_;

      void init();
      void cmdCb(const geometry_msgs::msg::Twist::SharedPtr msg);
      void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
      //publisher
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mavros_cmd_pub_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

      // Get clock for throttle log
      double kp_, kd_, ki_;
      double error_i_, error_d_, error_;
      double target_vy_;
      double compensate_vy_;
      rclcpp::Time last_odom_;
      geometry_msgs::msg::Twist cmd_vel_msg_;
      
  };
}

#endif  // ROBOT_SAFETY_WATCHDOG_HPP_