import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rlab_customized_ros_msg.action import MoveToPose
from geometry_msgs.msg import PoseStamped
import time

class AutonomousROVClient(Node):
    def __init__(self):
        super().__init__('autonomous_rov_client')
        self._action_client = ActionClient(self, MoveToPose, 'move_to_pose')
        
        # Create a timer that calls timerCb every second
        self.temperature_timer_ = self.create_timer(1.0, self.timerCb)
        self.counter = 0
        self._goal_handle = None  # Store the goal handle to use for canceling

    def timerCb(self):
        self.get_logger().info('Timer callback called')
        self.counter += 1
        # If the timer has been called more than 10 times, cancel the goal
        if self.counter > 30 and self._goal_handle is not None:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal cancel failed')

    def send_goal(self):
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = MoveToPose.Goal()
        goal_msg.target_pose = self.create_target_pose()

        self.get_logger().info(f'Sending goal: {goal_msg.target_pose}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def create_target_pose(self):
        # Define the target pose to reach a depth of -2 meters
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = -2.0  # Target depth included in the goal
        pose.pose.orientation.w = 1.0
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle  # Store the goal handle to use later for canceling
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.progress * 100:.2f}% complete')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().info('Goal failed')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    rov_client = AutonomousROVClient()
    rov_client.send_goal()
    rclpy.spin(rov_client)

if __name__ == '__main__':
    main()
