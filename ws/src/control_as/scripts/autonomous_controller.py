#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rlab_customized_ros_msg.action import MoveTo, SnailPattern


class ActionClientBase:
    def __init__(self, action_client):
        self.action_client = action_client
        self.goal_status = -1
        self._is_done = True
        self.goal_handle = None
        self.logger = rclpy.logging.get_logger(self.action_client._action_name)
        self.logger.info(f"{self.action_client._action_name}: Waiting for server...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error(f"{self.action_client._action_name}: Server unavailable.")
        else:
            self.logger.info(f"{self.action_client._action_name}: Server ready.")

    def execute_goal(self, goal):
        self.logger.info(f"Executing goal for {self.action_client._action_name}.")
        self._is_done = False
        self.goal_status = -1
        self.send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.warning(f"Goal for {self.action_client._action_name} rejected.")
            self._is_done = True
            return

        self.logger.info(f"Goal for {self.action_client._action_name} accepted.")
        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.goal_status = future.result().status
        self._is_done = True
        if self.goal_status == 4:
            self.logger.info(f"Goal for {self.action_client._action_name} succeeded.")
        elif self.goal_status == 6:
            self.logger.warning(f"Goal for {self.action_client._action_name} failed.")

    def feedback_callback(self, feedback_msg):
        self.logger.info(f"{self.action_client._action_name} feedback: {feedback_msg.feedback.progress * 100:.2f}%")

    def is_done(self):
        return self._is_done

    def is_succeeded(self):
        return self.goal_status == 4

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.logger.warning(f"Cancelling goal for {self.action_client._action_name}.")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.logger.info(f"Goal for {self.action_client._action_name} successfully cancelled.")
        else:
            self.logger.warning(f"Goal for {self.action_client._action_name} could not be cancelled.")
        self._is_done = True


class ROVAutonomousController(Node):
    def __init__(self):
        super().__init__('rov_autonomous_controller')

        # Initialize clients
        self.move_to_client = ActionClientBase(ActionClient(self, MoveTo, 'move_to'))
        self.snail_pattern_client = ActionClientBase(ActionClient(self, SnailPattern, 'snail_pattern'))

        # State Variables
        self.target_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.triggered = False
        self.fsm = "FSM_IDLE"
        self.pending_goal = False

        # Setup Subscriptions
        self.setup_subscriptions()

        # FSM Timer
        self.fsm_loop = self.create_timer(0.1, self.fsm_control_loop)

    def setup_subscriptions(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(State, 'mavros/state', self.state_callback, qos_profile)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_profile)
        self.create_subscription(String, '/task_command', self.task_command_callback, 10)

    def task_command_callback(self, msg):
        command = msg.data.lower().strip()
        if command == "start":
            self.triggered = True
            self.pending_goal = True
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

        if self.fsm == "FSM_IDLE" and self.triggered and self.pending_goal:
            self.fsm = "FSM_Send_MoveTo_Goal"
            self.get_logger().info("Trigger received, transitioning to FSM_Send_MoveTo_Goal")

        elif self.fsm == "FSM_Send_MoveTo_Goal":
            self.move_to_client.execute_goal(self.create_move_to_goal())
            self.get_logger().info(f"Sending MoveTo goal: {self.target_position}")
            self.fsm = "FSM_Wait_MoveTo_Goal"

        elif self.fsm == "FSM_Wait_MoveTo_Goal":
            if self.move_to_client.is_done():
                if self.move_to_client.is_succeeded():
                    self.get_logger().info("MoveTo goal succeeded. Transitioning to SnailPattern.")
                    self.fsm = "FSM_Send_SnailPattern_Goal"
                else:
                    self.get_logger().warn("MoveTo goal failed. Returning to IDLE.")
                    self.fsm = "FSM_IDLE"
                    self.pending_goal = False

        elif self.fsm == "FSM_Send_SnailPattern_Goal":
            self.snail_pattern_client.execute_goal(self.create_snail_pattern_goal())
            self.get_logger().info("Sending SnailPattern goal.")
            self.fsm = "FSM_Wait_SnailPattern_Goal"

        elif self.fsm == "FSM_Wait_SnailPattern_Goal":
            self.get_logger().info("Waiting for SnailPattern goal to finish...")
            if self.snail_pattern_client.is_done():
                if self.snail_pattern_client.is_succeeded():
                    self.get_logger().info("SnailPattern goal succeeded. Returning to IDLE.")
                else:
                    self.get_logger().warn("SnailPattern goal failed. Returning to IDLE.")
                self.fsm = "FSM_IDLE"
                self.pending_goal = False

        elif self.fsm == "FSM_ABORT":
            self.move_to_client.cancel_goal()
            self.snail_pattern_client.cancel_goal()
            self.triggered = False
            self.pending_goal = False
            self.fsm = "FSM_IDLE"

    def create_move_to_goal(self):
        """Create and return a MoveTo goal."""
        goal = MoveTo.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = self.target_position["x"]
        goal.target_pose.pose.position.y = self.target_position["y"]
        goal.target_pose.pose.position.z = self.target_position["z"]
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def create_snail_pattern_goal(self):
        """Create and return a SnailPattern goal."""
        goal = SnailPattern.Goal()
        goal.initial_side_length = 2.0
        goal.increment = 2.0
        goal.max_side_length = 4.0
        return goal


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
