#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

import time
import copy

class HeadOut(Node):
    
    def __init__(self):

        super().__init__('arm_and_guide')
        
        self.clock = self.get_clock()

        best_effort_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=3
        )

        sub_group = MutuallyExclusiveCallbackGroup()
        self.mavros_state_sub = self.create_subscription(State, 'mavros/state', self.mavrosStateCb, 3, callback_group=sub_group)
        self.mavros_state_ = []

        #Create service
        cg_service_group = MutuallyExclusiveCallbackGroup()
        #see: https://masoudir.github.io/mavros_tutorial/Chapter1_ArduRover_with_CLI/Step1_How_to_change_mode/
        self.mavros_mode_client = self.create_client(SetMode, 'mavros/set_mode', callback_group=cg_service_group)
        self.mavros_mode_req = SetMode.Request()
        self.mavros_mode_req.custom_mode = "GUIDED"

        self.mavros_arm_client = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=cg_service_group)
        self.mavros_arm_req = CommandBool.Request()
        self.mavros_arm_req.value = True

        #Main loop
        cg_fsm_loop = MutuallyExclusiveCallbackGroup()
        self.fsm_loop = self.create_timer(0.1, self.fsmControlLoop, callback_group=cg_fsm_loop)
        self.fsm = "FSM_Change_Mode"
        
        self.moving_forward_time = time.time()


    def mavrosStateCb(self, msg):
        self.mavros_state_ = copy.deepcopy(msg)

    def fsmControlLoop(self):

        if(not self.mavros_state_):
            return

        if(self.fsm == "FSM_Change_Mode"):
            if(not self.mavros_state_.mode == self.mavros_mode_req.custom_mode):
                self.get_logger().info("Change vehicle mode")
                future = self.mavros_mode_client.call_async(self.mavros_mode_req)
                while(rclpy.ok() and not future.done()):
                    time.sleep(0.2)
                time.sleep(0.5)
                if(self.mavros_state_.mode == self.mavros_mode_req.custom_mode):
                    self.get_logger().info("Vehicle is at " + self.mavros_mode_req.custom_mode + " mode")
                    self.fsm = "FSM_Try_Arming"
                else:
                    self.get_logger().info("Vehicle mode not change")
            else:
                self.get_logger().info("UUV is already in the state: %s" % self.mavros_state_.mode)
                self.fsm = "FSM_Try_Arming"

        elif(self.fsm == "FSM_Try_Arming"):
            if(not self.mavros_state_.armed):
                self.get_logger().info("Arm vehicle")
                future = self.mavros_arm_client.call_async(self.mavros_arm_req)
                while(rclpy.ok() and not future.done()):
                    time.sleep(0.2)
                time.sleep(0.5)
                if(self.mavros_state_.armed):
                    self.get_logger().info("Vehicle Armed")
                    self.fsm = "FSM_Armed"
                else:
                    self.get_logger().info("Vehicle not armed")
            else:
                self.fsm = "FSM_Armed"

def main(args=None):

    rclpy.init(args=args)

    HO = HeadOut()

    executor = MultiThreadedExecutor()
    executor.add_node(HO)
    executor.spin()

if __name__ == '__main__':
    main()