
#ifndef UUV_SYS_CORE_BASE_P2P_LOCAL_PLANNER_H
#define UUV_SYS_CORE_BASE_P2P_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>

#include <actionlib/server/simple_action_server.h>
#include <uuv_sys_core/P2PMoveBaseAction.h>

namespace uuv_sys_core {

  /**
   * @class BaseLocalPlanner
   * @brief Provides an interface for local planners used in navigation. All local planners written as plugins for the navigation stack must adhere to this interface.
   */
  class BaseP2PLocalPlanner{

    public:
      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return PlannerState based on the condition implemented
       */
      virtual PlannerState computeVelocityCommands(std::string traj_gen_name, geometry_msgs::Twist& cmd_vel) = 0;
      
      /**
       * @brief  Check if the goal position has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached() = 0;

      /**
       * @brief  Check the heading of the robot is pointing to the future plan
       * @return True if aligned, false otherwise
       */
      virtual bool isInitialHeadingAligned() = 0;

      /**
       * @brief  Check the shortest angle between goal and robot heading is aligned
       * @return True if aligned, false otherwise
       */
      virtual bool isGoalHeadingAligned() = 0;

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      virtual void initialize(std::string name, std::shared_ptr<tf2_ros::Buffer> tf2Buffer, uuv_sys_core::P2PMoveBaseActionServer* as) = 0;

      virtual geometry_msgs::TransformStamped getGlobalPose() = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseP2PLocalPlanner(){}

    protected:
      BaseP2PLocalPlanner(){}
  };
};  // namespace uuv_sys_core

#endif  // uuv_sys_core_BASE_P2P_LOCAL_PLANNER_H
