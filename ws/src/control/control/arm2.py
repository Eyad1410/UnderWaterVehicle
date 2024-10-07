import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist
import time


class ArmAndMoveClient(Node):
    def __init__(self):
        super().__init__('arm_and_move_client')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.vel_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

    def arm_vehicle(self):
        request = CommandBool.Request()
        request.value = True
        self.arm_client.call_async(request).result()

    def set_guided_mode(self):
        request = SetMode.Request()
        request.custom_mode = 'GUIDED'
        self.set_mode_client.call_async(request).result()

    def move_forward(self, duration=2):
        twist = Twist()
        twist.linear.x = 0.5
        start_time = time.time()
        while time.time() - start_time < duration:
            self.vel_publisher.publish(twist)
            time.sleep(0.1)

    def rotate_90_degrees(self, duration=2):
        twist = Twist()
        twist.angular.z = 0.5
        start_time = time.time()
        while time.time() - start_time < duration:
            self.vel_publisher.publish(twist)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    client = ArmAndMoveClient()

    client.arm_vehicle()
    client.set_guided_mode()

    for _ in range(4):  # Move in a square pattern
        client.move_forward(2)
        client.rotate_90_degrees(2)  # Adjust this duration based on vehicle's rotation speed

    rclpy.shutdown()


if __name__ == '__main__':
    main()

