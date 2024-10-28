#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

/*Fast triangulation of unordered point clouds*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

/*voxel*/
#include <pcl/filters/voxel_grid.h>

/*For edge markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*
for graph
type edge_t is inside here
*/
#include <set>

/*For distance calculation*/
#include <pcl/common/geometry.h>
#include <math.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*TF listener, although it is included in sensor.h*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

/*srv msg for make plan*/
#include "uuv_sys_core/action/get_plan.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <perception_3d/perception_3d_ros.h>

namespace global_planner
{

class Global_Planner : public rclcpp::Node {
    public:
      Global_Planner(const std::string& name);
      ~Global_Planner();

      void initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d);
   
    private:

      bool is_active(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> handle) const
      {
        return handle != nullptr && handle->is_active();
      }

      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const uuv_sys_core::action::GetPlan::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> goal_handle);

      void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> goal_handle);
      
      std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> current_handle_;

      rclcpp_action::Server<uuv_sys_core::action::GetPlan>::SharedPtr action_server_global_planner_;

      rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
      rclcpp::CallbackGroup::SharedPtr action_server_group_;
      
      rclcpp::Clock::SharedPtr clock_;

      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
      
      std::string global_frame_;
      std::string robot_frame_;

      /*
      Graph class:
      static: similar to static layer in costmap_2d
      dynamic: similar to obstacle layer in costmap_2d, this class require frequently clearing, therefore, keep it small
      */
      perception_3d::StaticGraph static_graph_ = perception_3d::StaticGraph();
      
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;

      /*cb*/
      rclcpp::TimerBase::SharedPtr perception_3d_check_timer_;
      void checkPerception3DThread();
      
      /*Func*/
      void makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> goal_handle);
      void cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal);
      void getCoveragedPath(nav_msgs::msg::Path& coveraged_path);
      void interpolatePath(nav_msgs::msg::Path& in_path, nav_msgs::msg::Path& ros_path);

    protected:
      std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tfl_;
};

}
