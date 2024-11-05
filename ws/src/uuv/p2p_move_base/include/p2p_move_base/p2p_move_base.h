
/*Debug*/
#include <chrono>
#include <p2p_move_base/p2p_fsm.h>

//@in enum state, the p_to_p_move_base is included
#include <uuv_sys_core/uuv_enum_states.h>

//@local planner
#include <local_planner/local_planner.h>

//@for call global planner action
#include "uuv_sys_core/action/get_plan.hpp"
//@for call recovery action
#include "uuv_sys_core/action/recovery_behaviors.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace p2p_move_base
{

class P2PMoveBase : public rclcpp::Node {

  public:

    P2PMoveBase(std::string name);
    ~P2PMoveBase();

    void initial(const std::shared_ptr<local_planner::Local_Planner>& lp);

  private:

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const uuv_sys_core::action::PToPMoveBase::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::PToPMoveBase>> goal_handle);

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::PToPMoveBase>> goal_handle);
    
    rclcpp_action::Server<uuv_sys_core::action::PToPMoveBase>::SharedPtr action_server_p2p_move_base_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::PToPMoveBase>> current_handle_;
    
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    rclcpp::CallbackGroup::SharedPtr action_server_group_;
    rclcpp::CallbackGroup::SharedPtr global_planner_client_group_;
    rclcpp::CallbackGroup::SharedPtr recovery_behaviors_client_group_;

    rclcpp::Clock::SharedPtr clock_;
    
    std::string name_;
    
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_cmd_vel_pub_;

    bool isQuaternionValid(const geometry_msgs::msg::Quaternion& q);

    void publishZeroVelocity();
    void publishVelocity(double vx, double vy, double angular_z);

    std::shared_ptr<p2p_move_base::FSM> FSM_;
    std::shared_ptr<local_planner::Local_Planner> LP_;

    void executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::PToPMoveBase>> goal_handle);

    bool executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::PToPMoveBase>> goal_handle);

    bool is_active(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<uuv_sys_core::action::PToPMoveBase>> handle) const
    {
      return handle != nullptr && handle->is_active();
    }
    
    rclcpp_action::Client<uuv_sys_core::action::GetPlan>::SharedPtr global_planner_client_ptr_;
    void global_planner_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<uuv_sys_core::action::GetPlan>::SharedPtr & goal_handle);
    void global_planner_client_result_callback(const rclcpp_action::ClientGoalHandle<uuv_sys_core::action::GetPlan>::WrappedResult & result);
    bool is_planning_;
    nav_msgs::msg::Path global_path_;
    void startGlobalPlanning();

    rclcpp_action::Client<uuv_sys_core::action::RecoveryBehaviors>::SharedPtr recovery_behaviors_client_ptr_;
    void recovery_behaviors_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<uuv_sys_core::action::RecoveryBehaviors>::SharedPtr & goal_handle);
    void recovery_behaviors_client_result_callback(const rclcpp_action::ClientGoalHandle<uuv_sys_core::action::RecoveryBehaviors>::WrappedResult & result);
    bool is_recoverying_;
    bool is_recoverying_succeed_;
    void startRecoveryBehaviors();


};



}//end of name space