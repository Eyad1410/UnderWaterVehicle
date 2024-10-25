#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rlab_customized_ros_msg.action import MoveToPose
from geometry_msgs.msg import PoseStamped
import time
from rclpy.executors import MultiThreadedExecutor

class MoveToPoseClient(Node):

    def __init__(self):
        super().__init__('move_to_pose_client')
        self.get_logger().info("Starting MoveToPose client")
        self._action_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.gh = None
        self.goal_sent = False
        self.goal_active = False

    def send_goal(self, target_pose):
        self.get_logger().info("Sending Goal")
        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose = target_pose
        goal_msg.repeat_count = 5  # Example repeat count

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info("Goal sent. Waiting for response")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.gh = goal_handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_sent = False
            self.goal_active = False
            return
        self.get_logger().info('Goal accepted :)')
        self.goal_active = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.goal_sent = True
    
    def get_result_callback(self, future):
        if future.result().status == 4:
            self.get_logger().info('Goal status: Succeeded')
        elif future.result().status == 6:
            self.get_logger().info('Goal status: Aborted')
        else:
            self.get_logger().info(f'Goal status: {future.result().status}')

        result = future.result().result
        if result.success:
            self.get_logger().info('Goal completed successfully')
        else:
            self.get_logger().info('Goal failed')
        
        # Reset the goal handle after completion
        self.gh = None
        self.goal_sent = False
        self.goal_active = False
        
    def cancel_goal(self):
        if self.gh and self.goal_active:
            self.get_logger().info('Attempting to cancel goal...')
            cancel_future = self.gh.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if cancel_response.accepted:
            self.get_logger().info('Goal cancel request accepted')
        else:
            self.get_logger().info('Goal cancel request rejected')
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveToPoseClient()
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)

    # Setting target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    target_pose.pose.position.x = 5.0
    target_pose.pose.position.y = 5.0
    target_pose.pose.position.z = -2.0
    target_pose.pose.orientation.w = 1.0

    count = 0
    a = time.time()
    b = True
    c = False
    already_pub = False
    turn_on_count = 0
    turn_off_count = 0
    
    while rclpy.ok():
        if time.time() - a > 30 and b:  # Wait 10 seconds before sending the goal
            if not action_client.goal_sent:
                action_client.send_goal(target_pose)  # Trigger the action with the target pose
                b = False
                c = True
                already_pub = False
                turn_on_count += 1
                action_client.get_logger().warn('>>>>>>>> Sending goal, count %d' % turn_on_count)
        else:
            if c and time.time() - a > 4 and not already_pub:
                if action_client.goal_active:
                    action_client.cancel_goal()
                    a = time.time()
                    b = True
                    already_pub = True
                    turn_off_count += 1
                    action_client.get_logger().warn('>>>>>>>> Canceling goal, total count %d' % turn_off_count)
        
        executor.spin_once(timeout_sec=0.2)     
    
    action_client.get_logger().warn('>>>>>>>> Exiting client node')

if __name__ == '__main__':
    main()


