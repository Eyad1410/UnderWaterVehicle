import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rlab_customized_ros_msg.action import MoveToPose
import time
from rclpy.executors import MultiThreadedExecutor

class ExampleMoveToPoseClient(Node):
    def __init__(self):
        super().__init__('example_trigger_client')
        self.get_logger().info("Starting trigger client")
        self._action_client = ActionClient(self, MoveToPose, 'move_to_pose')
<<<<<<< HEAD
        
        self.temperature_timer_ = self.create_timer(
            1.0, self.timerCb)
        self.counter = 0
    def timerCb(self):
        self.get_logger().info('Hi')
        self.counter = self.counter + 1
        if(self.counter>10):
            self._action_client.cancel_goal_async()
            
=======
        self.gh = ''
        self.goal_sent_time = None

>>>>>>> e3c0c4b (Latest move_to_pose action file & util_AS)
    def send_goal(self):
        self.get_logger().info("Sending Goal")
        goal_msg = MoveToPose.Goal()
        # Set target pose or other parameters for the MoveToPose action here
        goal_msg.target_pose.pose.position.z = -2.0  # Example value
        goal_msg.target_pose.pose.orientation.w = 1.0
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info("Goal sent. Waiting response")
        self.goal_sent_time = time.time()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.gh = goal_handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        if future.result().status == 4:
            self.get_logger().info('Goal status: Succeed')
        else:
            if future.result().status == 6: 
                self.get_logger().info('Goal status: ABORTED')
            else:
                pass
                # self.get_logger().info('Goal status: ' + str(future.result().status))

        # Additional handling of result if needed
        # rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    action_client = ExampleMoveToPoseClient()
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)
    b = True
    turn_on_count = 0
    turn_off_count = 0
    while rclpy.ok():
        current_time = time.time()
        if b:
            action_client.send_goal()             
            b = False
            turn_on_count += 1
            action_client.get_logger().warn('>>>>>>>> send goal, total count %d' % turn_on_count)
        else:
            if action_client.goal_sent_time and (current_time - action_client.goal_sent_time > 30):
                action_client.gh.cancel_goal_async() 
                b = True
                action_client.goal_sent_time = None
                turn_off_count += 1
                action_client.get_logger().warn('>>>>>>>> cancel goal, total count %d' % turn_off_count)
        executor.spin_once(timeout_sec=0.2)     
    action_client.get_logger().warn('>>>>>>>> 123')

if __name__ == '__main__':
    main()

