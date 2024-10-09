// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include "math.h" 
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
 
// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/convert.h"

#include "tf2_ros/transform_broadcaster.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;

class Cmd2Tf_Map2Baselink : public rclcpp::Node
{

  public:
    // Constructor creates a node named minimal_publisher. 
    // The published message count is initialized to 0.
    Cmd2Tf_Map2Baselink():Node("cmd_2_tf_map_2_baselink"){init();};
 
  private:
    
    rclcpp::Clock::SharedPtr clock_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_pub_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    void init();
  
    void cmdCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void tfPub();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fp_vrtk_pub_;
    

    /* tf2 */
    tf2::Transform tf2_init_;
    
    rclcpp::Time last_cmd_time_;
    
    geometry_msgs::msg::Twist cmd_;
    nav_msgs::msg::Odometry odom_, last_odom_;
    double theta_;
    double init_x_, init_y_, init_z_;
    std::string frame_id_, child_frame_id_;
};

void Cmd2Tf_Map2Baselink::init(){
  
  clock_ = this->get_clock();

  last_cmd_time_ = clock_->now();
  
  theta_ = 0.0;
  //TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  this->declare_parameter("frame_id", rclcpp::ParameterValue("map"));
  this->get_parameter("frame_id", frame_id_);
  RCLCPP_INFO(this->get_logger(), "frame_id_: %s" , frame_id_.c_str());

  this->declare_parameter("child_frame_id", rclcpp::ParameterValue("base_link"));
  this->get_parameter("child_frame_id", child_frame_id_);
  RCLCPP_INFO(this->get_logger(), "child_frame_id: %s" , child_frame_id_.c_str());

  this->declare_parameter("init_x", rclcpp::ParameterValue(0.0));
  this->get_parameter("init_x", init_x_);
  RCLCPP_INFO(this->get_logger(), "init_x: %.2f" , init_x_);

  this->declare_parameter("init_y", rclcpp::ParameterValue(0.0));
  this->get_parameter("init_y", init_y_);
  RCLCPP_INFO(this->get_logger(), "init_y: %.2f" , init_y_);

  this->declare_parameter("init_z", rclcpp::ParameterValue(0.0));
  this->get_parameter("init_z", init_z_);
  RCLCPP_INFO(this->get_logger(), "init_z: %.2f" , init_z_);

  odom_.pose.pose.position.x = init_x_;
  odom_.pose.pose.position.y = init_y_;
  odom_.pose.pose.position.z = init_z_;

  //Subscriber and Publisher
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 2, std::bind(&Cmd2Tf_Map2Baselink::cmdCb, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 2);
  fp_vrtk_pub_ = this->create_publisher<std_msgs::msg::String>("fort/vrtk", 2);

  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
  tf_pub_timer_ = this->create_wall_timer(20ms, std::bind(&Cmd2Tf_Map2Baselink::tfPub, this), timer_cb_group_);

}

void Cmd2Tf_Map2Baselink::tfPub(){

  double dt = (clock_->now() - last_cmd_time_).seconds();
  last_cmd_time_ = clock_->now();

  odom_.twist.twist.linear.x = (last_odom_.twist.twist.linear.x + cmd_.linear.x)/2.;
  odom_.twist.twist.angular.z = (last_odom_.twist.twist.angular.z + cmd_.angular.z)/2.;

  double dt_theta = (last_odom_.twist.twist.angular.z + cmd_.angular.z)/2.*dt;

  double base_x_vel = (last_odom_.twist.twist.linear.x + cmd_.linear.x)/2. * cos( dt_theta );
  double base_y_vel = (last_odom_.twist.twist.linear.x + cmd_.linear.x)/2. * sin( dt_theta );

  
  double dt_odom_x = ( base_x_vel * cos(theta_) - base_y_vel * sin(theta_) ) * dt;
  double dt_odom_y = ( base_x_vel * sin(theta_) + base_y_vel * cos(theta_) ) * dt;

  odom_.pose.pose.position.x += dt_odom_x;
  odom_.pose.pose.position.y += dt_odom_y;
  theta_ += dt_theta;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  std::string m_ns = this->get_namespace();
  if(m_ns.at(0) == '/'){
    m_ns.erase(0, 1);
  }
  std::string complete_frame_id = m_ns + "/" + frame_id_;;
  if(complete_frame_id.at(0) == '/'){
    complete_frame_id.erase(0, 1);
  }
  std::string complete_child_frame_id = m_ns + "/" + child_frame_id_;
  if(complete_child_frame_id.at(0) == '/'){
    complete_child_frame_id.erase(0, 1);
  }  
  odom_.header.frame_id = complete_frame_id;
  odom_.header.stamp = clock_->now();
  odom_.child_frame_id = complete_child_frame_id;
  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();
  
  geometry_msgs::msg::TransformStamped t;
  t.header = odom_.header;
  t.child_frame_id = complete_child_frame_id;
  t.transform.translation.x = odom_.pose.pose.position.x;
  t.transform.translation.y = odom_.pose.pose.position.y;
  t.transform.translation.z = odom_.pose.pose.position.z; 
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  
  tf_broadcaster_->sendTransform(t);
  odom_pub_->publish(odom_);
  last_odom_ = odom_;

  std_msgs::msg::String tmp_str;
  tmp_str.data = "11";
  fp_vrtk_pub_->publish(tmp_str);

  
}

void Cmd2Tf_Map2Baselink::cmdCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_ = (*msg);
}


// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<Cmd2Tf_Map2Baselink>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
