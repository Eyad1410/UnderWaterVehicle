#! /usr/bin/env python3

import rclpy


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import copy

#mavros
from mavros_msgs.srv import CommandBool

class UUVCommander(Node):
    
    def __init__(self):

        super().__init__('uuv_commander')
        
        #Create service
        action_client_group = MutuallyExclusiveCallbackGroup()
        self.mavros_arm_client = self.create_client(CommandBool, '/mavros/cmd/arming', callback_group=action_client_group)
        self.mavros_arm_req = CommandBool.Request()
        self.mavros_arm_req.value = True
        self.mavros_disarm_req = CommandBool.Request()
        self.mavros_disarm_req.value = False

        #Main loop
        cg_fsm_loop = MutuallyExclusiveCallbackGroup()
        self.fsm_loop = self.create_timer(0.1, self.fsmControlLoop, callback_group=cg_fsm_loop)


    def fsmControlLoop(self):
        future = self.mavros_arm_client.call_async(self.mavros_arm_req)
        while(rclpy.ok() and not future.done()):
            time.sleep(0.2)
        self.get_logger().info("UUV armed successfully")       
        pass


def main(args=None):

    rclpy.init(args=args)

    UUVC = UUVCommander()

    executor = MultiThreadedExecutor()
    executor.add_node(UUVC)
    executor.spin()

if __name__ == '__main__':
    main()