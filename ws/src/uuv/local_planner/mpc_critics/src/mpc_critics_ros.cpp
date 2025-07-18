
#include <mpc_critics/mpc_critics_ros.h>

namespace mpc_critics
{

MPC_Critics_ROS::MPC_Critics_ROS(const std::string& name):
  Node(name),
  name_(name),
  stacked_scoring_model_(NULL),
  model_loader_(name, "mpc_critics::ScoringModel")
{

}

void MPC_Critics_ROS::initial(){

  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  stacked_scoring_model_ = new StackedScoringModel(this->get_node_logging_interface(), tf2Buffer_);

  //@Start to load plugins
  this->declare_parameter("plugins", rclcpp::PARAMETER_STRING_ARRAY);
  rclcpp::Parameter plugins = this->get_parameter("plugins");
  plugins_ = plugins.as_string_array();
  for(auto i=plugins_.begin(); i!=plugins_.end(); i++){

    //@ get plugin type
    std::string get_plugin_type_str = (*i)+".plugin"; //plugins.plugin, ex: map.plugin
    this->declare_parameter(get_plugin_type_str, rclcpp::ParameterValue(""));
    rclcpp::Parameter plugin_type_param = this->get_parameter(get_plugin_type_str);
    std::string plugin_type_str = plugin_type_param.as_string();

    std::string get_plugin2traj_type_str = (*i)+".trajectory_generator"; //plugins.plugin, ex: map.plugin
    this->declare_parameter(get_plugin2traj_type_str, rclcpp::ParameterValue(""));
    rclcpp::Parameter plugin2traj_type_param = this->get_parameter(get_plugin2traj_type_str);
    std::string plugin2traj_type_str = plugin2traj_type_param.as_string();

    RCLCPP_INFO(this->get_logger(), "Trajectory generator: %s using ---> %s", plugin2traj_type_str.c_str(), plugin_type_str.c_str());  

    std::shared_ptr<mpc_critics::ScoringModel> plugin = model_loader_.createSharedInstance(plugin_type_str);
    stacked_scoring_model_->addPluginByTraj(plugin2traj_type_str, plugin);
    //Pass shared data for Sensor to share customized data betrween plugins
    plugin->initialize((*i), shared_from_this());
  }
}

MPC_Critics_ROS::~MPC_Critics_ROS()
{
  delete stacked_scoring_model_;
}

void MPC_Critics_ROS::scoreTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& one_traj){

  stacked_scoring_model_->scoreTrajectory(traj_gen_name, one_traj);
}

void MPC_Critics_ROS::updateSharedData(){
  stacked_scoring_model_->getSharedDataPtr()->updateData();
}

}//end of name space