
#ifndef TRAJECTORY_SHARED_DATA_H_
#define TRAJECTORY_SHARED_DATA_H_

#include "rclcpp/rclcpp.hpp"

/*TF listener*/
/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

/*For robot state*/
#include "nav_msgs/msg/odometry.hpp"


/*tf to affine*/
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>

#include <math.h>

namespace trajectory_generators
{

class TrajectoryGeneratorSharedData{
  public:

    TrajectoryGeneratorSharedData(std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer):
    tf2Buffer_(m_tf2Buffer),
    current_allowed_max_linear_speed_(-1.0)
    {};
    
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer(){return tf2Buffer_;}

    geometry_msgs::msg::TransformStamped robot_pose_;
    geometry_msgs::msg::TransformStamped prune_end_pose_base_link_frame_;
    nav_msgs::msg::Odometry robot_state_;
    nav_msgs::msg::Path prune_plan_;
    double current_allowed_max_linear_speed_;

    std::string global_frame_, base_frame_;


    void updateGoalatRobotFrame(){

      geometry_msgs::msg::TransformStamped pose_prune_end;
      pose_prune_end.header = prune_plan_.header;
      pose_prune_end.transform.translation.x = prune_plan_.poses.back().pose.position.x;
      pose_prune_end.transform.translation.y = prune_plan_.poses.back().pose.position.y;
      pose_prune_end.transform.translation.z = prune_plan_.poses.back().pose.position.z;

      pose_prune_end.transform.rotation.x = prune_plan_.poses.back().pose.orientation.x;
      pose_prune_end.transform.rotation.y = prune_plan_.poses.back().pose.orientation.y;
      pose_prune_end.transform.rotation.z = prune_plan_.poses.back().pose.orientation.z;
      pose_prune_end.transform.rotation.w = prune_plan_.poses.back().pose.orientation.w;

      Eigen::Affine3d pruneend2map_af3 = tf2::transformToEigen(pose_prune_end).inverse();
      Eigen::Affine3d map2baselink_af3 = tf2::transformToEigen(robot_pose_);

      Eigen::Affine3d pruneend2baselink_af3 = pruneend2map_af3*map2baselink_af3;
      Eigen::Affine3d baselink2pruneend_af3 = pruneend2baselink_af3.inverse();

      prune_end_pose_base_link_frame_ = tf2::eigenToTransform (baselink2pruneend_af3);

    }
  
  private:

    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_; 
    

};


}//end of name space

#endif