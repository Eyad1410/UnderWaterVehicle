#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import traceback
import time
import os
from datetime import datetime

class LogManager():
    
    def __init__(self, m_logger, m_name):
        self.logger = m_logger
        self.report = []
        self.time_key = {}
        self.log_dir = os.getenv("ROS_LOG_DIR")
        if(self.log_dir):
            date_time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.file_writer = open(self.log_dir + "/" + m_name + "_" + date_time_str + ".log", "w", buffering=1)
        else:
            self.file_writer = open("/demofile3.txt", "w")

    def write_log(self, to_write):
        date_time_str = datetime.utcnow().strftime('%F %T.%f')[:-3]
        complete_log = date_time_str + " >>> " + to_write + "\n"
        self.file_writer.write(complete_log)
        self.file_writer.flush()

    def log(self, level ,to_write, show_msg = False, write_db=False, err_code=''):
        if(level == 'info'):
            self.logger.info(to_write)
            self.write_log(to_write)
        elif(level == 'warn'):
            self.logger.warn(to_write)
            self.write_log(to_write)
        elif(level == 'error'):
            self.logger.error(to_write)
            self.write_log(to_write)
            
            
    def log_throttle(self, level, duration, key ,to_write):

        if(key in self.time_key):
            last_write_time = self.time_key[key]
        else:
            self.time_key.update({key:time.time()})
            last_write_time = time.time() - duration - 1.0

        if(level == 'info'):
            if(time.time() - last_write_time >duration):
                self.logger.info(to_write)
                self.write_log(to_write)
                self.time_key.update({key:time.time()})
        elif(level == 'warn'):
            if(time.time() - last_write_time >duration):
                self.logger.warn(to_write)
                self.write_log(to_write)
                self.time_key.update({key:time.time()})
        elif(level == 'err'):
            if(time.time() - last_write_time >duration):
                self.logger.error(to_write)
                self.write_log(to_write)
                self.time_key.update({key:time.time()})
    
    def report_event_log(self):
        
        '''
        define costumized rules
        '''
        pass
                
    def queue_report(self,describe):
        self.report.append(describe)



        