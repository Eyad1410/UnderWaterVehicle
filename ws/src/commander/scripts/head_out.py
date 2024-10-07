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

        super().__init__('head_out_node')
        
        self.clock = self.get_clock()

        best_effort_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=3
        )

        sub_group = MutuallyExclusiveCallbackGroup()
        self.mavros_state_sub = self.create_subscription(State, 'mavros/state', self.mavrosStateCb, 3, callback_group=sub_group)
        self.mavros_state_ = State()
        self.mavros_odometry = self.create_subscription(Odometry, 'mavros/global_position/local', self.odometryCb, qos_profile=best_effort_qos_profile, callback_group=sub_group)
        self.mavros_odom_ = Odometry()

        #Create service
        cg_service_group = MutuallyExclusiveCallbackGroup()
        #see: https://masoudir.github.io/mavros_tutorial/Chapter1_ArduRover_with_CLI/Step1_How_to_change_mode/
        self.mavros_mode_client = self.create_client(SetMode, 'mavros/set_mode', callback_group=cg_service_group)
        self.mavros_mode_req = SetMode.Request()
        self.mavros_mode_req.custom_mode = "GUIDED"

        self.mavros_arm_client = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=cg_service_group)
        self.mavros_arm_req = CommandBool.Request()
        self.mavros_arm_req.value = True
        
        #velocity commander
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 2)

        #Main loop
        cg_fsm_loop = MutuallyExclusiveCallbackGroup()
        self.fsm_loop = self.create_timer(0.1, self.fsmControlLoop, callback_group=cg_fsm_loop)
        self.fsm = "FSM_Change_Mode"
        
        self.moving_forward_time = time.time()

    def odometryCb(self, msg):
        self.mavros_odom_ = copy.deepcopy(msg)

    def mavrosStateCb(self, msg):
        self.mavros_state_ = copy.deepcopy(msg)

    def fsmControlLoop(self):

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
        
        elif(self.fsm == "FSM_Armed"):
            self.fsm = "FSM_Descending"
            self.get_logger().info("Start to descend vehicle")

        elif(self.fsm == "FSM_Descending"):
            cmd_msg = TwistStamped()
            if(self.mavros_odom_.pose.pose.position.z <= -5.0):
                self.fsm = "FSM_Moving_Forward"
                self.get_logger().info("Start to move forward vehicle")
            else:
                cmd_msg.header.frame_id = "base_link"
                cmd_msg.header.stamp = self.clock.now().to_msg()
                cmd_msg.twist.linear.z = -0.5
            self.cmd_vel_pub.publish(cmd_msg)

        elif(self.fsm == "FSM_Moving_Forward"):
            cmd_msg = TwistStamped()
            if(time.time() - self.moving_forward_time>=100.0):
                self.fsm = "FSM_Stop"
            else:
                cmd_msg.header.frame_id = "base_link"
                cmd_msg.header.stamp = self.clock.now().to_msg()
                cmd_msg.twist.linear.x = -0.5
            self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):

    rclpy.init(args=args)

    HO = HeadOut()

    executor = MultiThreadedExecutor()
    executor.add_node(HO)
    executor.spin()

if __name__ == '__main__':
    main()