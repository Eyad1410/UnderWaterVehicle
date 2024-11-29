#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup



class ActionClientBase:
    """
    A base class for handling ROS 2 Action Clients.

    Attributes:
        node (Node): The ROS 2 node using this action client.
        action_client (ActionClient): The ROS 2 action client.
        goal_status (int): Current status of the goal.
        _is_done (bool): Whether the goal execution is completed.
        goal_handle (GoalHandle): Handle for the current goal.
    """
    def __init__(self, node, action_type, action_name):
        """
        Initializes the ActionClientBase.

        Args:
            node (Node): The ROS 2 node using this action client.
            action_type (Type): The type of the action (e.g., MoveTo, SnailPattern).
            action_name (str): The name of the action server.
        """
        self.node = node
        self.action_client = ActionClient(node, action_type, action_name, callback_group=ReentrantCallbackGroup())
        self.goal_status = -1
        self._is_done = True
        self.goal_handle = None

        self.node.get_logger().info(f"{action_name}: Waiting for server...")
        self.action_client.wait_for_server()
        self.node.get_logger().info(f"{action_name}: Server ready.")
    
    def execute_goal(self, goal):
        """
        Sends a goal to the action server for execution.

        Args:
            goal (Goal): The goal to be executed.
        """
        self.node.get_logger().info(f"{self.__class__.__name__}: Executing goal.")
        self._is_done = False
        self.goal_status = -1
        self.send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback for when the server responds to the goal.

        Args:
            future (Future): The future containing the goal response.
        """
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
        """
        Callback for when the server provides the result of the goal.

        Args:
            future (Future): The future containing the result.
        """
        self.goal_status = future.result().status
        self._is_done = True
        if self.goal_status == 4:
            self.node.get_logger().info(f"{self.__class__.__name__}: Goal succeeded.")
        elif self.goal_status == 6:
            self.node.get_logger().info(f"{self.__class__.__name__}: Goal failed.")

    def feedback_callback(self, feedback_msg):
        """
        Callback for receiving feedback from the action server.

        Args:
            feedback_msg (Feedback): The feedback message.
        """
        self.node.get_logger().info(f"{self.__class__.__name__} feedback: {feedback_msg.feedback.progress * 100:.2f}%")

    def is_done(self):
        """
        Checks if the goal execution is completed.

        Returns:
            bool: True if the goal is done, False otherwise.
        """
        return self._is_done

    def is_succeeded(self):
        """
        Checks if the goal execution succeeded.

        Returns:
            bool: True if the goal succeeded, False otherwise.
        """
        return self.goal_status == 4

    def cancel_goal(self):
        """
        Cancels the currently active goal.
        """
        if self.goal_handle is not None:
            self.node.get_logger().warn(f"{self.__class__.__name__}: Cancelling goal.")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """
        Callback for when the server acknowledges the cancel request.

        Args:
            future (Future): The future containing the cancel response.
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().warn(f"{self.__class__.__name__}: Goal successfully cancelled.")
        else:
            self.node.get_logger().warn(f"{self.__class__.__name__}: Goal could not be cancelled.")
        self._is_done = True

    def reset(self):
        """
        Resets the state of the action client.
        """
        self.goal_status = -1
        self._is_done = True
        self.goal_handle = None
        self.node.get_logger().info(f"Action client {self.action_client._action_name} has been reset.")



