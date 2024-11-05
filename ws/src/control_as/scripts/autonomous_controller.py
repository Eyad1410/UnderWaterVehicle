#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.action import ActionClient

from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from rlab_customized_ros_msg.action import MoveTo, SnailPattern  # Action types

import time
import copy

class ROVAutonomousController(Node):
    
    def __init__(self):
        super().__init__('rov_control_node')
        
        self.clock = self.get_clock()

        # QoS Profile
        best_effort_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=3
        )

        sub_group = MutuallyExclusiveCallbackGroup()
        self.rov_state_subscription = self.create_subscription(
            State, 'mavros/state', self.state_callback, 3, callback_group=sub_group)
        self.rov_state = None

        # Service groups
        cg_service_group = MutuallyExclusiveCallbackGroup()
        self.rov_set_mode_client = self.create_client(SetMode, 'mavros/set_mode', callback_group=cg_service_group)
        self.rov_set_mode_req = SetMode.Request()
        self.rov_set_mode_req.custom_mode = "GUIDED"

        self.rov_arm_client = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=cg_service_group)
        self.rov_arm_req = CommandBool.Request()
        self.rov_arm_req.value = True

        # Initialize Action Clients with ReentrantCallbackGroup to allow concurrent calls
        action_group = ReentrantCallbackGroup()
        self.move_to_goal_client = ActionClient(self, MoveTo, 'move_to', callback_group=action_group)
        self.snail_pattern_client = ActionClient(self, SnailPattern, 'snail_pattern', callback_group=action_group)

        # Main loop group
        cg_fsm_loop = MutuallyExclusiveCallbackGroup()
        self.fsm_loop = self.create_timer(0.1, self.fsm_control_loop, callback_group=cg_fsm_loop)
        self.fsm = "FSM_Set_Mode_GUIDED"

    def state_callback(self, msg):
        # Callback to update MAVROS state
        self.rov_state = copy.deepcopy(msg)

    def fsm_control_loop(self):
        # FSM to manage arming and GUIDED mode setup
        if not self.rov_state:
            return

        if self.fsm == "FSM_Set_Mode_GUIDED":
            if self.rov_state.mode != self.rov_set_mode_req.custom_mode:
                self.get_logger().info("Changing vehicle mode to GUIDED")
                future = self.rov_set_mode_client.call_async(self.rov_set_mode_req)
                rclpy.spin_until_future_complete(self, future)
                if self.rov_state.mode == self.rov_set_mode_req.custom_mode:
                    self.get_logger().info("Vehicle is now in GUIDED mode")
                    self.fsm = "FSM_Arm_Vehicle"
                else:
                    self.get_logger().info("Failed to change vehicle mode")
            else:
                self.get_logger().info(f"Vehicle is already in {self.rov_state.mode} mode")
                self.fsm = "FSM_Arm_Vehicle"

        elif self.fsm == "FSM_Arm_Vehicle":
            if not self.rov_state.armed:
                self.get_logger().info("Arming vehicle")
                future = self.rov_arm_client.call_async(self.rov_arm_req)
                rclpy.spin_until_future_complete(self, future)
                if self.rov_state.armed:
                    self.get_logger().info("Vehicle is now armed")
                    self.fsm = "FSM_Ready"
                else:
                    self.get_logger().info("Failed to arm vehicle")
            else:
                self.get_logger().info("Vehicle is already armed")
                self.fsm = "FSM_Ready"

        elif self.fsm == "FSM_Ready":
            # System is armed and in GUIDED mode, ready for actions
            self.get_logger().info("System is ready. Waiting for commands or actions.")
            self.fsm = "FSM_IDLE"

        elif self.fsm == "FSM_IDLE":
            # In IDLE state, waiting for future goals or commands
            pass

def main(args=None):
    rclpy.init(args=args)
    
    rov_controller = ROVAutonomousController()
    executor = MultiThreadedExecutor()
    executor.add_node(rov_controller)
    executor.spin()

    # Cleanup
    rov_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

