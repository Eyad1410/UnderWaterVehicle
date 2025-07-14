#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rlab_customized_ros_msg.action import MoveTo, SnailPattern
from cv_bridge import CvBridge
import cv2
import time
from image_stitching import Image_Stitching  # stitching module
import os

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

        # Image capture vars
        self.capturing_images = False
        self.bridge = CvBridge()
        self.last_capture_time = time.time()
        self.image_count = 0
        self.image_buffer = []
        self.image_directory = "/UnderWaterVehicle/ws/src/control_as/images"  # already exists

        # Use node from action_client to create subscription
        self.image_sub = self.action_client._node.create_subscription(Image, "/rgb_camera", self.image_callback, 10)

    def execute_goal(self, goal):
        self.logger.info(f"Executing goal for {self.action_client._action_name}.")
        self._is_done = False
        self.goal_status = -1
        self.send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        if self.action_client._action_name == "snail_pattern":
            self.capturing_images = True
            self.logger.info("Image capturing started.")

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

        if self.action_client._action_name == "snail_pattern":
            self.capturing_images = False
            self.logger.info("Image capturing stopped.")

    def image_callback(self, msg):
        """Captures images every second while SnailPattern executes, and stitches every 2."""
        if self.capturing_images:
            current_time = time.time()
            if current_time - self.last_capture_time >= 0.25:
                self.last_capture_time = current_time
                try:
                    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    image_path = os.path.join(self.image_directory, f"image_{self.image_count:04d}.jpg")
                    cv2.imwrite(image_path, image)
                    self.logger.info(f"✅ Saved image: {image_path}")
                    self.image_buffer.append(image)

                    # Stitch every 2 images (pairwise stitching)
                    if len(self.image_buffer) == 2:
                        stitcher = Image_Stitching()
                        pano = stitcher.blending(*self.image_buffer)
                        if pano is not None:
                            pano_path = os.path.join(self.image_directory, f"panorama_{self.image_count:04d}.jpg")
                            cv2.imwrite(pano_path, pano)
                            self.logger.info(f"🧵 Panorama saved: {pano_path}")
                        else:
                            self.logger.warn("❌ Stitching failed.")
                        self.image_buffer = []

                    self.image_count += 1

                except Exception as e:
                    self.logger.error(f"❌ Failed to save image: {e}")

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

        # Pass only 1 argument to ActionClientBase constructor
        self.move_to_client = ActionClientBase(ActionClient(self, MoveTo, 'move_to'))
        self.snail_pattern_client = ActionClientBase(ActionClient(self, SnailPattern, 'snail_pattern'))

        self.target_position = {"x": 0.0, "y": 0.0, "z": -2.0}
        self.triggered = False
        self.fsm = "FSM_IDLE"
        self.pending_goal = False

        self.setup_subscriptions()
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
                    self.get_logger().info("SnailPattern goal succeeded. Starting hierarchical panorama merge.")
                    self.merge_all_panoramas()
                    self.get_logger().info("Hierarchical panorama merging complete. Returning to IDLE.")
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
        goal = MoveTo.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = self.target_position["x"]
        goal.target_pose.pose.position.y = self.target_position["y"]
        goal.target_pose.pose.position.z = self.target_position["z"]
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def create_snail_pattern_goal(self):
        goal = SnailPattern.Goal()
        goal.initial_side_length = 1.0
        goal.increment = 1.0
        goal.max_side_length = 2.0
        return goal

    # Hierarchical stitching methods
    def hierarchical_stitch(self, stitcher, images):
        if len(images) == 1:
            return images[0]

        merged_images = []
        i = 0
        while i < len(images):
            if i + 1 < len(images):
                pano = stitcher.blending(images[i], images[i + 1])
                if pano is None:
                    pano = images[i]  # fallback if stitching fails
                merged_images.append(pano)
                i += 2
            else:
                merged_images.append(images[i])
                i += 1
        return self.hierarchical_stitch(stitcher, merged_images)

    def merge_all_panoramas(self):
        image_folder = "/UnderWaterVehicle/ws/src/control_as/images"
        pano_files = sorted([os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.startswith("panorama_")])
        panoramas = [cv2.imread(f) for f in pano_files]

        if len(panoramas) == 0:
            self.get_logger().warn("No panoramas found to merge.")
            return

        stitcher = Image_Stitching()
        final_pano = self.hierarchical_stitch(stitcher, panoramas)

        if final_pano is not None:
            final_path = os.path.join(image_folder, "final_panorama.jpg")
            cv2.imwrite(final_path, final_pano)
            self.get_logger().info(f"Final panorama saved at {final_path}")
        else:
            self.get_logger().warn("Final panorama stitching failed.")


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






