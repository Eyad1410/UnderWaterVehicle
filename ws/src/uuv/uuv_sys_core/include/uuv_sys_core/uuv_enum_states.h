#ifndef UUV_SYS_CORE_ENUM_STATES_H
#define UUV_SYS_CORE_ENUM_STATES_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>

#include "rclcpp_action/rclcpp_action.hpp"
#include "uuv_sys_core/action/p_to_p_move_base.hpp"

namespace uuv_sys_core {

  typedef rclcpp_action::Server<uuv_sys_core::action::PToPMoveBase> PToPMoveBaseActionServer;

  enum PlannerState {
    TF_FAIL,
    PRUNE_PLAN_FAIL,
    ALL_TRAJECTORIES_FAIL,
    PERCEPTION_MALFUNCTION,
    TRAJECTORY_FOUND,
    PATH_BLOCKED_WAIT,
    PATH_BLOCKED_REPLANNING
  };

  enum RecoveryState {
    RECOVERY_BEHAVIOR_NOT_FOUND,
    INTERRUPT_BY_CANCEL,
    INTERRUPT_BY_NEW_GOAL,
    RECOVERY_DONE,
    RECOVERY_FAIL
  };

  class uuv_enum_states
  {
  private:
    /* data */
  public:
    uuv_enum_states();
    ~uuv_enum_states();
  };
  
};  // namespace uuv_sys_core

#endif  // uuv_sys_core_BASE_P2P_LOCAL_PLANNER_H
