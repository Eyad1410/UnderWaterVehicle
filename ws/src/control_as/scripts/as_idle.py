#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from c_log_manager import LogManager

from rclpy.node import Node

from rlab_customized_ros_msg.action import Idle

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
#for quaternion transformation
from tf_utils import TF_Utils

import time
import datetime
import threading

class IdleActionServer(Node):

    def __init__(self):
        super().__init__('action_server_idle_test')

        self.Log_Manager = LogManager(self.get_logger(), self.get_name())

        cg1 = ReentrantCallbackGroup()
        cg2 = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Idle,
            'as_idle_test',
            callback_group=cg1,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)


        self.previous_goal_handle = None
        self.current_goal_handle = None
        self.abort_previoud_goal = False

        self.get_logger().info('action_server_idle_test is ready')

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request

        if(self.current_goal_handle is not None and self.previous_goal_handle is None):
            if(self.current_goal_handle.is_active):
                self.get_logger().warn('set the abort flag for previous_goal_handle')
                self.abort_previoud_goal = True
                self.previous_goal_handle = self.current_goal_handle
                #self.get_logger().warn('double trigger this action server,reject and abort pervious one') # if open these two line, double trigger will reject and cancel current one
                #return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        
        self.get_logger().info('Start action idle test')
        self.get_logger().info('receive idle time requirement: %.2f' % self.goal.idle_time)
        goal_idle_time = self.goal.idle_time  # get the client requirement
        #goal_idle_time = 5
        self.current_goal_handle = goal_handle
        thread_id = threading.get_ident()
        statemachine = 'idle_start'
        start_time = time.time()
        time.sleep(0.3)

        while(rclpy.ok()):

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result = Idle.Result()
                return result
            
            # prevent the multi thread 
            if(self.abort_previoud_goal):
                self.get_logger().error('abort pervious one')
                self.previous_goal_handle.abort()
                result_1 = Idle.Result()
                self.abort_previoud_goal = False
                self.previous_goal_handle = None
                return result_1
            
            if(statemachine == 'idle_start'):
                if(time.time() - start_time > goal_idle_time):
                    goal_handle.succeed()
                    result = Idle.Result()
                    self.get_logger().info('End action idle')
                    return result
            
            self.Log_Manager.log_throttle("info", 1, "idle_action", "idling...")
            #self.get_logger().info('thread id ----> %s' % thread_id)   #check for current thread id
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    idle_action_server = IdleActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(idle_action_server)
    executor.spin()

if __name__ == '__main__':
    main()
