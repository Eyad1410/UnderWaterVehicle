#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rclpy
from rclpy.action import ActionClient

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

class EasyClient():
    def __init__(self, m_action_client, m_name, m_action_msg, m_logger, m_clock):
        self.name = m_name
        self.logger = m_logger
        self.clock = m_clock
        self.as_client = m_action_client
        self.target = m_action_msg.Goal()
        self.goal_reset = m_action_msg.Goal()
        self.feedback = m_action_msg.Feedback()
        self.goal_status = -1
        self.logger.warn(self.name + ": Start to wait")    
        self.as_client.wait_for_server()
        self.logger.warn(self.name + ": Got server")  
        self.isdone = True
        
        
    def isDone(self):
        return self.isdone

    def goal_response_callback(self, future):
        self.isdone = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info(self.name + ": Goal rejected")
            self.isdone = True
            return

        self.logger.info(self.name + ": Goal accepted")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def isSucceed(self):
        if(self.goal_status == 4):
            return True
        else:
            return False

    def get_result_callback(self, future):
        
        #self.logger.info(self.name + ": Result: {0}".format(future.result()))
        self.goal_status = future.result().status
        self.action_result = future.result().result
        self.isdone = True
        # Indicates status has not been properly set.
        #int8 STATUS_UNKNOWN   = 0

        # The goal has been accepted and is awaiting execution.
        #int8 STATUS_ACCEPTED  = 1

        # The goal is currently being executed by the action server.
        #int8 STATUS_EXECUTING = 2

        # The client has requested that the goal be canceled and the action server has
        # accepted the cancel request.
        #int8 STATUS_CANCELING = 3

        # The goal was achieved successfully by the action server.
        #int8 STATUS_SUCCEEDED = 4

        # The goal was canceled after an external request from an action client.
        #int8 STATUS_CANCELED  = 5

        # The goal was terminated by the action server without an external request.
        #int8 STATUS_ABORTED   = 6

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback
        #self.logger.info('Received feedback: {0}'.format(feedback.number_of_poses_remaining))
        self.server_hb = self.clock.now().to_msg()
        pass
        

    def execute_goal(self):
        self.logger.info(self.name + ": execute goal.")
        self.isdone = False
        self.server_hb = self.clock.now().to_msg()
        self.goal_status = -1
        
        self.send_goal_future = self.as_client.send_goal_async(self.target, feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        try:
            if(not self.isdone):
                self.send_goal_future.result().cancel_goal_async()
        except:
            pass   
 
