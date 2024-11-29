#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rlab_customized_ros_msg.action import MoveTo, SnailPattern
import json
import math
import time

class ActionClientBase:
    def __init__(self, node, action_type, action_name):
        self.node = node
        self.action_client = ActionClient(node, action_type, action_name, callback_group=ReentrantCallbackGroup())
        self.goal_status = -1
        self._is_done = True
        self.goal_handle = None
        self.node.get_logger().info(f"{action_name}: Waiting for server...")
        self.action_client.wait_for_server()
        self.node.get_logger().info(f"{action_name}: Server ready.")
    
    def execute_goal(self, goal):
        self.node.get_logger().info(f"{self.__class__.__name__}: Executing goal.")
        self._is_done = False
        self.goal_status = -1
        self.send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info(f"{self.__class__.__name__}: Goal rejected")
            self._is_done = True
            return

        self.node.get_logger().info(f"{self.__class__.__name__}: Goal accepted")
        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.goal_status = future.result().status
        self._is_done = True
        if self.goal_status == 4:
            self.node.get_logger().info(f"{self.__class__.__name__}: Goal succeeded.")
        elif self.goal_status == 6:
            self.node.get_logger().info(f"{self.__class__.__name__}: Goal failed.")

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info(f"{self.__class__.__name__} feedback: {feedback_msg.feedback.progress * 100:.2f}%")

    def is_done(self):
        return self._is_done

    def is_succeeded(self):
        return self.goal_status == 4

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.node.get_logger().warn(f"{self.__class__.__name__}: Cancelling goal.")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().warn(f"{self.__class__.__name__}: Goal successfully cancelled.")
        else:
            self.node.get_logger().warn(f"{self.__class__.__name__}: Goal could not be cancelled.")
        self._is_done = True

    def reset(self):
        """Reset the state of the action client."""
        self.goal_status = -1
        self._is_done = True
        self.goal_handle = None
        self.node.get_logger().info(f"Action client {self.action_client._action_name} has been reset.")


class MoveToClient(ActionClientBase):
    def __init__(self, node):
        super().__init__(node, MoveTo, 'move_to')
    
    def send_goal(self, x, y, z):
        goal = MoveTo.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = 1.0
        self.execute_goal(goal)


class SnailPatternClient(ActionClientBase):
    def __init__(self, node):
        super().__init__(node, SnailPattern, 'snail_pattern')

    def send_goal(self, initial_side_length, increment, max_side_length):
        goal = SnailPattern.Goal()
        goal.initial_side_length = initial_side_length
        goal.increment = increment
        goal.max_side_length = max_side_length
        self.execute_goal(goal)


class ROVAutonomousController(Node):
    def __init__(self):
        super().__init__('rov_autonomous_controller')

        # Initialize clients
        self.move_to_client = MoveToClient(self)
        self.snail_pattern_client = SnailPatternClient(self)

        # State Variables
        self.positions = self.load_positions("/UnderWaterVehicle/ws/src/control_as/scripts/positions.json")
        self.current_position_index = 0
        self.triggered = False
        self.fsm = "FSM_IDLE"

        # Setup Subscriptions and Services
        self.setup_subscriptions_and_services()

        # FSM Timer
        self.fsm_loop = self.create_timer(0.1, self.fsm_control_loop)

    def setup_subscriptions_and_services(self):
        """Setup subscriptions and services."""
        # State and Battery Monitoring
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Fix QoS mismatch
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(State, 'mavros/state', self.state_callback, qos_profile)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_profile)

        # Task Commands
        self.create_subscription(String, '/task_command', self.task_command_callback, 10)

        # MAVROS Services
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')

    def load_positions(self, file_path):
        """Load positions from a JSON file."""
        try:
            with open(file_path, 'r') as file:
                positions = json.load(file).get("positions", [])
                self.get_logger().info(f"Loaded {len(positions)} positions from {file_path}")
                return positions
        except Exception as e:
            self.get_logger().error(f"Failed to load positions from {file_path}: {e}")
            return []

    def task_command_callback(self, msg):
        command = msg.data.lower().strip()
        if command == "start":
            if not self.triggered:
                self.triggered = True
                self.get_logger().info("Received 'start' command. FSM triggered.")
        elif command == "abort":
            self.triggered = False
            self.fsm = "FSM_ABORT"
            self.get_logger().warn("Received 'abort' command. Aborting active tasks.")

    def state_callback(self, msg):
        self.rov_state = msg

    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage

    def fsm_control_loop(self):
        self.get_logger().info(f"Current FSM State: {self.fsm}")

        if self.fsm == "FSM_IDLE" and self.triggered:
            self.fsm = "FSM_Send_MoveTo_Goal"
            self.get_logger().info("Trigger received, transitioning to FSM_Send_MoveTo_Goal")

        elif self.fsm == "FSM_Send_MoveTo_Goal":
            if self.current_position_index < len(self.positions):
                position = self.positions[self.current_position_index]
                self.move_to_client.send_goal(position["x"], position["y"], position["z"])
                self.get_logger().info(f"Sending MoveTo goal: {position}")
                self.fsm = "FSM_Wait_MoveTo_Goal"
            else:
                self.get_logger().info("All positions processed. Returning to IDLE.")
                self.fsm = "FSM_IDLE"

        elif self.fsm == "FSM_Wait_MoveTo_Goal":
            if self.move_to_client.is_done():
                if self.move_to_client.is_succeeded():
                    self.get_logger().info("MoveTo goal succeeded. Transitioning to SnailPattern.")
                    self.fsm = "FSM_Send_SnailPattern_Goal"
                else:
                    self.get_logger().warn("MoveTo goal failed. Proceeding to next position.")
                    self.current_position_index += 1  # Increment index
                    self.fsm = "FSM_Send_MoveTo_Goal"  # Trigger the next goal

        elif self.fsm == "FSM_Send_SnailPattern_Goal":
            self.snail_pattern_client.send_goal(2.0, 2.0, 4.0)  # Adjust values as needed
            self.get_logger().info("Sending SnailPattern goal.")
            self.fsm = "FSM_Wait_SnailPattern_Goal"

        elif self.fsm == "FSM_Wait_SnailPattern_Goal":
            self.get_logger().info(f"Waiting for SnailPattern goal to finish... FSM state: {self.fsm}")
            if self.snail_pattern_client.is_done():
                if self.snail_pattern_client.is_succeeded():
                    self.get_logger().info("SnailPattern goal succeeded. Moving to next position.")
                    self.current_position_index += 1  # Increment index after completing the pattern
                else:
                    self.get_logger().warn("SnailPattern goal failed. Proceeding to next position.")
                self.fsm = "FSM_Send_MoveTo_Goal"  # Ensure transition to next goal

        elif self.fsm == "FSM_ABORT":
            self.get_logger().warn("Aborting all active goals.")
            self.move_to_client.cancel_goal()
            self.snail_pattern_client.cancel_goal()
            self.triggered = False
            self.fsm = "FSM_IDLE"


def main(args=None):
    rclpy.init(args=args)
    node = ROVAutonomousController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



