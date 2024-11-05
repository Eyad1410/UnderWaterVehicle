#include <global_planner/global_planner.h>

using namespace std::chrono_literals;

namespace global_planner
{

Global_Planner::Global_Planner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse Global_Planner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const uuv_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Global_Planner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Global_Planner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Global_Planner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void Global_Planner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d){

  perception_3d_ros_ = perception_3d;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;
  
  perception_3d_check_timer_ = this->create_wall_timer(100ms, std::bind(&Global_Planner::checkPerception3DThread, this), action_server_group_);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);


  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&Global_Planner::cbClickedPoint, this, std::placeholders::_1), sub_options);

  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<uuv_sys_core::action::GetPlan>(
    this,
    "/get_plan",
    std::bind(&Global_Planner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Global_Planner::handle_cancel, this, std::placeholders::_1),
    std::bind(&Global_Planner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

Global_Planner::~Global_Planner(){

  //perception_3d_ros_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  action_server_global_planner_.reset();
}

void Global_Planner::checkPerception3DThread(){
  

}


void Global_Planner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  nav_msgs::msg::Path coveraged_path;
  coveraged_path.header.frame_id = global_frame_;
  getCoveragedPath(coveraged_path);
  pub_path_->publish(coveraged_path);

}

void Global_Planner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::GetPlan>> goal_handle){

  auto result = std::make_shared<uuv_sys_core::action::GetPlan::Result>();

  nav_msgs::msg::Path coveraged_path;
  coveraged_path.header.frame_id = global_frame_;
  getCoveragedPath(coveraged_path);

  nav_msgs::msg::Path interpolated_coveraged_path;
  interpolatePath(coveraged_path, interpolated_coveraged_path);

  pub_path_->publish(interpolated_coveraged_path);

  result->path = interpolated_coveraged_path;
  goal_handle->succeed(result);
  

}

void Global_Planner::getCoveragedPath(nav_msgs::msg::Path& coveraged_path){

  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to get robot pose: %s", e.what());
  }
  
  //@ generate a straight line to 0,0
  double delta_x = 0.0 - start.pose.position.x;
  double delta_y = 0.0 - start.pose.position.y;
  for(double i=0;i<sqrt(delta_x*delta_x + delta_y*delta_y);i+=0.05){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = global_frame_;
    ps.pose.position.x = start.pose.position.x + i*(delta_x/sqrt(delta_x*delta_x + delta_y*delta_y));
    ps.pose.position.y = start.pose.position.y + i*(delta_y/sqrt(delta_x*delta_x + delta_y*delta_y));
    ps.pose.position.z = start.pose.position.z;
    ps.pose.orientation.w = 1.0;
    coveraged_path.poses.push_back(ps);
  }
  double last_x = 0.0;
  double last_y = 0.0;
  double x_shrink = 2.0;
  double y_shrink = 2.0;
  double patch_length = 20.0;
  int pivot_x = 0;
  int pivot_y = 0;
  int pivot_i = 0;
  while(patch_length - pivot_i*x_shrink > x_shrink){
    double x_len = patch_length - pivot_i*x_shrink;
    double y_len = patch_length - pivot_i*y_shrink;
    for(double x=pivot_x*x_shrink; x<=x_len; x+=0.5){
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = global_frame_;
      ps.pose.position.x = x;
      ps.pose.position.y = last_y;
      ps.pose.position.z = start.pose.position.z;
      ps.pose.orientation.w = 1.0;
      coveraged_path.poses.push_back(ps);
      last_x = x;
    }
    for(double y=pivot_y*y_shrink; y<=y_len; y+=0.5){
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = global_frame_;
      ps.pose.position.x = last_x;
      ps.pose.position.y = y;
      ps.pose.position.z = start.pose.position.z;
      ps.pose.orientation.w = 1.0;
      coveraged_path.poses.push_back(ps);
      last_y = y;
    }
    pivot_x++;
    for(double x=last_x; x>=pivot_x*x_shrink; x-=0.5){
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = global_frame_;
      ps.pose.position.x = x;
      ps.pose.position.y = last_y;
      ps.pose.position.z = start.pose.position.z;
      ps.pose.orientation.w = 1.0;
      coveraged_path.poses.push_back(ps);
      last_x = x;
    }
    pivot_y++;
    for(double y=last_y; y>=pivot_y*y_shrink; y-=0.5){
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = global_frame_;
      ps.pose.position.x = last_x;
      ps.pose.position.y = y;
      ps.pose.position.z = start.pose.position.z;
      ps.pose.orientation.w = 1.0;
      coveraged_path.poses.push_back(ps);
      last_y = y;
    }
    pivot_i++;
  }
}

void Global_Planner::interpolatePath(nav_msgs::msg::Path& in_path, nav_msgs::msg::Path& ros_path){

  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();


  for(auto it=0;it<in_path.poses.size()-1;it++){
    geometry_msgs::msg::PoseStamped pst;
    pst = in_path.poses[it];

    geometry_msgs::msg::PoseStamped next_pst;
    if(it<in_path.poses.size()-1){
      next_pst = in_path.poses[it+1];
    }

    double vx,vy,vz;
    vx = next_pst.pose.position.x - pst.pose.position.x;
    vy = next_pst.pose.position.y - pst.pose.position.y;
    vz = next_pst.pose.position.z - pst.pose.position.z;
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();


    pst.pose.orientation.x = q.getX();
    pst.pose.orientation.y = q.getY();
    pst.pose.orientation.z = q.getZ();
    pst.pose.orientation.w = q.getW();     
    if(std::isnan(pst.pose.orientation.x) || std::isnan(pst.pose.orientation.y) || std::isnan(pst.pose.orientation.z) || std::isnan(pst.pose.orientation.w)){
      pst.pose.orientation.x = 0.0;
      pst.pose.orientation.y = 1.0;
      pst.pose.orientation.z = 0.0;
      pst.pose.orientation.w = 0.0;    
    }

    //@Interpolation to make global plan smoother and better resolution for local planner
    geometry_msgs::msg::PoseStamped pst_inter_polate = pst;
    if(it<in_path.poses.size()-1){
      ros_path.poses.push_back(pst);
      geometry_msgs::msg::PoseStamped last_pst = pst;
      for(double step=0.05;step<0.99;step+=0.05){

        pst_inter_polate.pose.position.x = pst.pose.position.x + vx*step;
        pst_inter_polate.pose.position.y = pst.pose.position.y + vy*step;
        pst_inter_polate.pose.position.z = pst.pose.position.z + vz*step;
        double dx = pst_inter_polate.pose.position.x-last_pst.pose.position.x;
        double dy = pst_inter_polate.pose.position.y-last_pst.pose.position.y;
        double dz = pst_inter_polate.pose.position.z-last_pst.pose.position.z;
        if(sqrt(dx*dx+dy*dy+dz*dz)>0.1){
          ros_path.poses.push_back(pst_inter_polate);
          last_pst = pst_inter_polate;
        }
        
      }
    }
    else{
      ros_path.poses.push_back(pst);
    }
    
  }
}

}