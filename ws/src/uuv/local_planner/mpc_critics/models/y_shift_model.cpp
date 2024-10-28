
#include <mpc_critics/y_shift_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::YShiftModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

YShiftModel::YShiftModel(){
  return;
  
}

void YShiftModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}


double YShiftModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(shared_data_->prune_plan_.poses.empty() || traj.getPointsSize()<2){
    return -4.0;  
  }

  geometry_msgs::msg::PoseStamped initial_traj_pose = traj.getPoint(0);
  geometry_msgs::msg::PoseStamped last_prune_plan_pose = shared_data_->prune_plan_.poses.back();
  geometry_msgs::msg::PoseStamped first_prune_plan_pose = shared_data_->prune_plan_.poses.front();

  //@ create tf pose for affine computation
  geometry_msgs::msg::TransformStamped tf_initial_traj_pose, tf_last_prune_plan_pose, tf_first_prune_plan_pose;
  tf_initial_traj_pose.header.frame_id = shared_data_->global_frame_;
  tf_initial_traj_pose.child_frame_id = shared_data_->base_frame_;
  tf_initial_traj_pose.transform.translation.x = initial_traj_pose.pose.position.x;
  tf_initial_traj_pose.transform.translation.y = initial_traj_pose.pose.position.y;
  tf_initial_traj_pose.transform.translation.z = initial_traj_pose.pose.position.z;
  tf_initial_traj_pose.transform.rotation = initial_traj_pose.pose.orientation;
  //@ tf of last_prune_plan_pose
  tf_last_prune_plan_pose.header.frame_id = shared_data_->global_frame_;
  tf_last_prune_plan_pose.child_frame_id = shared_data_->base_frame_;
  tf_last_prune_plan_pose.transform.translation.x = last_prune_plan_pose.pose.position.x;
  tf_last_prune_plan_pose.transform.translation.y = last_prune_plan_pose.pose.position.y;
  tf_last_prune_plan_pose.transform.translation.z = last_prune_plan_pose.pose.position.z;
  tf_last_prune_plan_pose.transform.rotation = last_prune_plan_pose.pose.orientation;  
  //@ tf of first prune plane pose
  tf_first_prune_plan_pose.header.frame_id = shared_data_->global_frame_;
  tf_first_prune_plan_pose.child_frame_id = shared_data_->base_frame_;
  tf_first_prune_plan_pose.transform.translation.x = first_prune_plan_pose.pose.position.x;
  tf_first_prune_plan_pose.transform.translation.y = first_prune_plan_pose.pose.position.y;
  tf_first_prune_plan_pose.transform.translation.z = first_prune_plan_pose.pose.position.z;
  tf_first_prune_plan_pose.transform.rotation = first_prune_plan_pose.pose.orientation;  

  //tf_initial_traj_pose
  Eigen::Affine3d initial_traj_pose_af3 = tf2::transformToEigen(tf_initial_traj_pose);
  //@ inverse initial_traj_pose_af3 to get initial_traj_pose->global
  initial_traj_pose_af3 = initial_traj_pose_af3.inverse();
  //@ we have global->last_prune_plan_pose
  Eigen::Affine3d last_prune_plan_pose_af3 = tf2::transformToEigen(tf_last_prune_plan_pose);
  Eigen::Affine3d first_prune_plan_pose_af3 = tf2::transformToEigen(tf_first_prune_plan_pose);

  //@ So we have initial_traj_pose to last_prune_plan_pose
  Eigen::Affine3d pose_difference_af3 = initial_traj_pose_af3*last_prune_plan_pose_af3;
  geometry_msgs::msg::TransformStamped tf_pose_difference = tf2::eigenToTransform(pose_difference_af3);

  //@ So we have initial_traj_pose to first_prune_plan_pose
  Eigen::Affine3d pose_difference_af3_2 = initial_traj_pose_af3*first_prune_plan_pose_af3;
  geometry_msgs::msg::TransformStamped tf_pose_difference_2 = tf2::eigenToTransform(pose_difference_af3_2);

  double r,y,p;
  tf2::Quaternion q;
  tf2::convert(tf_pose_difference.transform.rotation , q);
  tf2::Matrix3x3(q).getEulerYPR(y,p,r);

  y = std::fmod((y+3.1416),3.1416);
  double distance = 0.0;

  if(fabs(y)<=0.1745){
    double y_moving = traj.yv_;
    double distance_last = fabs(y_moving - tf_pose_difference.transform.translation.y);
    double distance_first = fabs(y_moving - tf_pose_difference_2.transform.translation.y);
    distance = std::max(distance_last, distance_first);
  }

  else
    distance = 0.0;

  return (weight_*distance);
}

}//end of name space