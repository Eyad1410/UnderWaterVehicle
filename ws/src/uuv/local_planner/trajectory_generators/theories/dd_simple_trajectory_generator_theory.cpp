
#include <trajectory_generators/dd_simple_trajectory_generator_theory.h>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::DDSimpleTrajectoryGeneratorTheory, trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

DDSimpleTrajectoryGeneratorTheory::DDSimpleTrajectoryGeneratorTheory(){
  return;
}

void DDSimpleTrajectoryGeneratorTheory::onInitialize(){

  //@initialize trajectory generator
  limits_ = std::make_shared<trajectory_generators::DDTrajectoryGeneratorLimits>();
  
  node_->declare_parameter(name_ + ".min_vel_x", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_x", limits_->min_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_x: %.2f", limits_->min_vel_x);

  node_->declare_parameter(name_ + ".max_vel_x", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_x", limits_->max_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_x: %.2f", limits_->max_vel_x);

  node_->declare_parameter(name_ + ".min_vel_theta", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".min_vel_theta", limits_->min_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_theta: %.2f", limits_->min_vel_theta);

  node_->declare_parameter(name_ + ".max_vel_theta", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_theta", limits_->max_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_theta: %.2f", limits_->max_vel_theta);

  node_->declare_parameter(name_ + ".acc_lim_x", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".acc_lim_x", limits_->acc_lim_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_x: %.2f", limits_->acc_lim_x);

  node_->declare_parameter(name_ + ".acc_lim_theta", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".acc_lim_theta", limits_->acc_lim_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_theta: %.2f", limits_->acc_lim_theta);

  node_->declare_parameter(name_ + ".prune_forward", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name_ + ".prune_forward", limits_->prune_forward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_forward: %.2f", limits_->prune_forward);

  node_->declare_parameter(name_ + ".prune_backward", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".prune_backward", limits_->prune_backward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_backward: %.2f", limits_->prune_backward);

  //@deceleration allow the robot to consider the deceleration based on the current_speed/deceleration_ratio
  //@for example, if current speed of robot is 1.2 then the considered min_vel will be 0.6 instead of current_speed-dt*acc
  node_->declare_parameter(name_ + ".deceleration_ratio", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".deceleration_ratio", limits_->deceleration_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "deceleration_ratio: %.2f", limits_->deceleration_ratio);

  if(limits_->min_vel_x<0)
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "The min velocity of the robot should be positive!");

  /*Motor constraint*/
  node_->declare_parameter(name_ + ".max_motor_shaft_rpm", rclcpp::ParameterValue(3000.0));
  node_->get_parameter(name_ + ".max_motor_shaft_rpm", limits_->max_motor_shaft_rpm);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_motor_shaft_rpm: %.2f", limits_->max_motor_shaft_rpm);

  node_->declare_parameter(name_ + ".wheel_diameter", rclcpp::ParameterValue(0.15));
  node_->get_parameter(name_ + ".wheel_diameter", limits_->wheel_diameter);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "wheel_diameter: %.2f", limits_->wheel_diameter);

  node_->declare_parameter(name_ + ".gear_ratio", rclcpp::ParameterValue(30.0));
  node_->get_parameter(name_ + ".gear_ratio", limits_->gear_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "gear_ratio: %.2f", limits_->gear_ratio);

  node_->declare_parameter(name_ + ".robot_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".robot_radius", limits_->robot_radius);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "robot_radius: %.2f", limits_->robot_radius);

  //@initial params
  params_ = std::make_shared<trajectory_generators::DDTrajectoryGeneratorParams>();

  node_->declare_parameter(name_ + ".controller_frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".controller_frequency", params_->controller_frequency);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "controller_frequency: %.2f", params_->controller_frequency);

  node_->declare_parameter(name_ + ".sim_time", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".sim_time", params_->sim_time);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_time: %.2f", params_->sim_time);

  node_->declare_parameter(name_ + ".linear_x_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".linear_x_sample", params_->linear_x_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_x_sample: %.2f", params_->linear_x_sample);

  node_->declare_parameter(name_ + ".angular_z_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".angular_z_sample", params_->angular_z_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_z_sample: %.2f", params_->angular_z_sample);

  node_->declare_parameter(name_ + ".sim_granularity", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".sim_granularity", params_->sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_granularity: %.2f", params_->sim_granularity);

  node_->declare_parameter(name_ + ".angular_sim_granularity", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".angular_sim_granularity", params_->angular_sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_sim_granularity: %.2f", params_->angular_sim_granularity);

  //@ parse cuboid
  /*
  for(int i=1;i<9;i++){
    std::string s = ".cuboid.p" + std::to_string(i);

    node_->declare_parameter(name_ + s, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param = node_->get_parameter(name_ + s);
    auto p = cuboid_param.as_double_array();
    pcl::PointXYZ pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    params_->cuboid.push_back(pt);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid %.2f, %.2f, %.2f", pt.x, pt.y, pt.z);
  }
  */
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<double> p;

  node_->declare_parameter(name_ + ".cuboid.flb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flb= node_->get_parameter(name_ + ".cuboid.flb");
  p = cuboid_flb.as_double_array();
  pcl::PointXYZ pt_flb;
  pt_flb.x = p[0];pt_flb.y = p[1];pt_flb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid flb: %.2f, %.2f, %.2f", pt_flb.x, pt_flb.y, pt_flb.z);

  node_->declare_parameter(name_ + ".cuboid.frb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frb= node_->get_parameter(name_ + ".cuboid.frb");
  p = cuboid_frb.as_double_array();
  pcl::PointXYZ pt_frb;
  pt_frb.x = p[0];pt_frb.y = p[1];pt_frb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid frb: %.2f, %.2f, %.2f", pt_frb.x, pt_frb.y, pt_frb.z);

  node_->declare_parameter(name_ + ".cuboid.flt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flt= node_->get_parameter(name_ + ".cuboid.flt");
  p = cuboid_flt.as_double_array();
  pcl::PointXYZ pt_flt;
  pt_flt.x = p[0];pt_flt.y = p[1];pt_flt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid flt: %.2f, %.2f, %.2f", pt_flt.x, pt_flt.y, pt_flt.z);

  node_->declare_parameter(name_ + ".cuboid.frt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frt= node_->get_parameter(name_ + ".cuboid.frt");
  p = cuboid_frt.as_double_array();
  pcl::PointXYZ pt_frt;
  pt_frt.x = p[0];pt_frt.y = p[1];pt_frt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid frt: %.2f, %.2f, %.2f", pt_frt.x, pt_frt.y, pt_frt.z);

  node_->declare_parameter(name_ + ".cuboid.blb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blb= node_->get_parameter(name_ + ".cuboid.blb");
  p = cuboid_blb.as_double_array();
  pcl::PointXYZ pt_blb;
  pt_blb.x = p[0];pt_blb.y = p[1];pt_blb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid blb: %.2f, %.2f, %.2f", pt_blb.x, pt_blb.y, pt_blb.z);

  node_->declare_parameter(name_ + ".cuboid.brb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brb= node_->get_parameter(name_ + ".cuboid.brb");
  p = cuboid_brb.as_double_array();
  pcl::PointXYZ pt_brb;
  pt_brb.x = p[0];pt_brb.y = p[1];pt_brb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid brb: %.2f, %.2f, %.2f", pt_brb.x, pt_brb.y, pt_brb.z);

  node_->declare_parameter(name_ + ".cuboid.blt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blt= node_->get_parameter(name_ + ".cuboid.blt");
  p = cuboid_blt.as_double_array();
  pcl::PointXYZ pt_blt;
  pt_blt.x = p[0];pt_blt.y = p[1];pt_blt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid blt: %.2f, %.2f, %.2f", pt_blt.x, pt_blt.y, pt_blt.z);

  node_->declare_parameter(name_ + ".cuboid.brt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brt= node_->get_parameter(name_ + ".cuboid.brt");
  p = cuboid_brt.as_double_array();
  pcl::PointXYZ pt_brt;
  pt_brt.x = p[0];pt_brt.y = p[1];pt_brt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid brt: %.2f, %.2f, %.2f", pt_brt.x, pt_brt.y, pt_brt.z);

  params_->cuboid.push_back(pt_blb);
  params_->cuboid.push_back(pt_brb);
  params_->cuboid.push_back(pt_blt);
  params_->cuboid.push_back(pt_flb);
  params_->cuboid.push_back(pt_brt);
  params_->cuboid.push_back(pt_frt);
  params_->cuboid.push_back(pt_flt);
  params_->cuboid.push_back(pt_frb);
  //@ push the point by following sequence, because when doing the point in cuboid test we leverage blb/brb/blt/flb
  //
  //          -------
  //         /|    /|
  //        / |   / |
  //     blt------- |
  //        | /flb| /
  //        |/    |/
  //     blb-------brb

  if(params_->cuboid.size()!=8){
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "Cuboid is essential.");
  }

}

void DDSimpleTrajectoryGeneratorTheory::initialise(){
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits_->max_vel_theta;
  double min_vel_th = -1.0 * max_vel_th;
  Eigen::Vector3f acc_lim = limits_->getAccLimits();
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits_->min_vel_x;
  double max_vel_x = limits_->max_vel_x;


  // if sampling number is zero in any dimension, we don't generate samples generically
  if (params_->linear_x_sample * params_->angular_z_sample > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

    // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
    double sim_period = 1.0/params_->controller_frequency;
    
    // allow max speed to be changed by speed zone/perception features
    if(shared_data_->current_allowed_max_linear_speed_>0.0){
      max_vel_x = std::min(max_vel_x, shared_data_->current_allowed_max_linear_speed_);
    }
    max_vel[0] = std::min(max_vel_x, shared_data_->robot_state_.twist.twist.linear.x + acc_lim[0] * sim_period);
    max_vel[1] = std::min(0.2, shared_data_->robot_state_.twist.twist.linear.y + acc_lim[0] * sim_period);
    max_vel[2] = std::min(max_vel_th, shared_data_->robot_state_.twist.twist.angular.z + acc_lim[2] * sim_period);

    min_vel[0] = std::max(min_vel_x, shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio);
    min_vel[1] = std::max(-0.2, shared_data_->robot_state_.twist.twist.linear.y/limits_->deceleration_ratio);
    min_vel[2] = std::max(min_vel_th, shared_data_->robot_state_.twist.twist.angular.z - acc_lim[2] * sim_period);
    
    // because the speed zone might introduce huge deceleration than robot kinematic
    // which will cause max_vel[0]<min_vel[0]
    // the min admissible will be shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio
    // because the robot will never decelerate enough to meet shared_data_->current_allowed_max_linear_speed_
    if(max_vel[0]<min_vel[0]){
      min_vel[0] = shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio;
      max_vel[0] = shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio;
    }

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    trajectory_generators::VelocityIterator x_it(min_vel[0], max_vel[0], params_->linear_x_sample);
    trajectory_generators::VelocityIterator y_it(-0.1, 0.1, 5);
    trajectory_generators::VelocityIterator th_it(min_vel[2], max_vel[2], params_->angular_z_sample);
    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          sample_params_.push_back(vel_samp);
        }
        th_it.reset();
      }
      y_it.reset();
    }
  }    
}

bool DDSimpleTrajectoryGeneratorTheory::isMotorConstraintSatisfied(Eigen::Vector3f& vel_samp){
  double vr,vl;
  vr = vel_samp[0] + limits_->robot_radius * vel_samp[2];
  vl = vel_samp[0] - limits_->robot_radius * vel_samp[2];
  double rpm_r, rpm_l;
  rpm_r = vr * limits_->gear_ratio * 60./3.1415926/limits_->wheel_diameter;
  rpm_l = vl * limits_->gear_ratio * 60./3.1415926/limits_->wheel_diameter;
  if(fabs(rpm_r)>=limits_->max_motor_shaft_rpm || fabs(rpm_l)>=limits_->max_motor_shaft_rpm)
    return false;
  return true;
}


bool DDSimpleTrajectoryGeneratorTheory::hasMoreTrajectories(){
  return next_sample_index_ < sample_params_.size();
}

bool DDSimpleTrajectoryGeneratorTheory::nextTrajectory(base_trajectory::Trajectory& _traj){
  bool result = false;
  /*
  Because generateTrajectory will return false when sample params are not satisfied, 
  the function will just sweep without generating any trajectory
  */
  bool generated_once = false;
  if (hasMoreTrajectories()) {
    
    if (generateTrajectory(
        sample_params_[next_sample_index_],
        _traj)) {
      result = true;
      generated_once = true;
    }
    else{
      _traj.resetPoints();
    }
    
  }
  else{
    if(!generated_once)
      RCLCPP_ERROR(node_->get_logger().get_child(name_), "None of trajectory is generated, maybe acc is too small or check the generateTrajectory function.");
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool DDSimpleTrajectoryGeneratorTheory::generateTrajectory(
      Eigen::Vector3f sample_target_vel,
      base_trajectory::Trajectory& traj) {

  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);
  double vmag = fabs(sample_target_vel[0]);
  double eps = 1e-4;
  traj.cost_ = 0.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();

  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  if ((limits_->min_vel_x >= 0 && vmag + eps < limits_->min_vel_x) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) {
    return false;
  }
  // make sure we do not exceed max diagonal (x+y) translational velocity (if set)
  if (limits_->max_vel_x >=0 && vmag - eps > limits_->max_vel_x) {
    return false;
  }

  int num_steps;

  //compute the number of steps we must take along this trajectory to be "safe"
  double sim_time_distance = vmag * params_->sim_time; // the distance the robot would travel in sim_time if it did not change velocity
  double sim_time_angle = fabs(sample_target_vel[2]) * params_->sim_time; // the angle the robot would rotate in sim_time
  num_steps =
      ceil(std::max(sim_time_distance / params_->sim_granularity,
          sim_time_angle / params_->angular_sim_granularity));
  

  if (num_steps == 0) {
    return false;
  }

  //compute a timestep
  double dt = params_->sim_time / num_steps;
  traj.time_delta_ = dt;

  //compute a timestep
  //double dt = 0.1;
  //int num_steps = 30;
  //traj.time_delta_ = dt;

  
  Eigen::Vector3f loop_vel;

  // assuming sample_vel is our target velocity within acc limits for one timestep
  loop_vel = sample_target_vel;
  traj.xv_     = sample_target_vel[0];
  traj.yv_     = sample_target_vel[1];
  traj.thetav_ = sample_target_vel[2];

  /*We first create trajectory based on robot_frame, then we use affine to transform it to global frame*/
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  //simulate the trajectory and check for collisions, updating costs along the way
  for (int i = 0; i < num_steps; ++i) {

    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);
    //y incurred by x speed and rotation
    double incur_weight = pos[2]*pos[2]*pos[2]*pos[2];
    incur_weight = std::min(incur_weight, 1.0);
    loop_vel[1] = loop_vel[0] * cos(1.5707963 + pos[2]) * incur_weight;
    /*transform back to global frame*/
    Eigen::Affine3d trans_gbl2traj_af3;

    
    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(pos[2], Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = pos[0];
    trans_b2traj_af3.translation().y() = pos[1];
    
    /*
    tf2::Quaternion tf2_rotation;
    tf2_rotation.setRPY( 0, 0, pos[2]); 
    tf2_rotation.normalize();
    geometry_msgs::TransformStamped trans_b2traj;
    trans_b2traj.transform.translation.x = pos[0];
    trans_b2traj.transform.translation.y = pos[1];
    trans_b2traj.transform.rotation.x = tf2_rotation.x();
    trans_b2traj.transform.rotation.x = tf2_rotation.y();
    trans_b2traj.transform.rotation.x = tf2_rotation.z();
    trans_b2traj.transform.rotation.x = tf2_rotation.w();
    Eigen::Affine3d trans_b2traj_af3 = tf2::transformToEigen(trans_b2traj);
    */
    trans_gbl2traj_af3 = pos_af3*trans_b2traj_af3;
    geometry_msgs::msg::TransformStamped trans_gbl2traj_ = tf2::eigenToTransform (trans_gbl2traj_af3);
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = shared_data_->robot_pose_.header;
    ros_pose.pose.position.x = trans_gbl2traj_.transform.translation.x;
    ros_pose.pose.position.y = trans_gbl2traj_.transform.translation.y;
    ros_pose.pose.position.z = trans_gbl2traj_.transform.translation.z;
    ros_pose.pose.orientation = trans_gbl2traj_.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> pc_out;
    pcl::transformPointCloud(params_->cuboid, pc_out, trans_gbl2traj_af3);
    
    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(pc_out, cuboid_min_max.first, cuboid_min_max.second);

    if(!traj.addPoint(ros_pose, pc_out, cuboid_min_max)){
      return false;
    }

  } // end for simulation steps
  //if(fabs(pos[1])>0.01){
  //  traj.yv_ = pos[1]/params_->sim_time;
  //}
  return true; // trajectory has at least one point
}

Eigen::Vector3f DDSimpleTrajectoryGeneratorTheory::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(1.5707963 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(1.5707963 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}
}//end of name space