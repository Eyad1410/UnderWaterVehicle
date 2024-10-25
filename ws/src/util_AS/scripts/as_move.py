#! /usr/bin/env python3
            self.get_logger().info('ROV armed successfully')
            return True
        else:
            self.get_logger().error('Failed to arm ROV')
            return False

    def set_mode(self, mode='GUIDED'):
        while not self.set_mode_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Mode set to {mode} successfully')
            return True
        else:
            self.get_logger().error(f'Failed to set mode to {mode}')
            return False

    def move_to_depth(self, target_depth):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.z = target_depth
        pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Moving to depth: {target_depth} meters')
        for _ in range(50):  # Simulate moving to the depth over 5 seconds
            if self.current_goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during depth movement')
                self.current_goal_handle.canceled()
                return False

            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            time.sleep(0.1)

        return True

    def rotate_to_yaw(self, goal_handle, target_yaw, max_duration=10.0):
        self.get_logger().info(f'Rotating to yaw: {math.degrees(target_yaw)} degrees')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = math.cos(target_yaw * 0.5)
        pose.pose.orientation.z = math.sin(target_yaw * 0.5)

        start_time = time.time()
        while time.time() - start_time < max_duration:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during yaw rotation')
                goal_handle.canceled()
                return False

            pose.header.stamp = self.get_clock().now().to_msg()
            self.position_publisher.publish(pose)
            time.sleep(0.1)

        return True

    def execute_snail_pattern(self):
        self.get_logger().info('Executing snail pattern')
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.3

        for _ in range(100):  # Simulate executing the snail pattern over 10 seconds
            if self.current_goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during snail pattern execution')
                self.current_goal_handle.canceled()
                return False

            self.velocity_publisher.publish(twist)
            time.sleep(0.1)

        self.stop_movement()
        return True

    def stop_movement(self):
        self.get_logger().info('Stopping movement')
        self.velocity_publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    move_to_pose_action_server = MoveToPoseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(move_to_pose_action_server)
    executor.spin()

if __name__ == '__main__':
    main()



