#! /usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import os

def readBags():

    ref_x = 3288233.0
    ref_y = 4745894.0
    ref_z = 2700951.0

    bags_folder = '/gps'
    dir_list = os.listdir(bags_folder)
    np_ecef_path_list = []
    for dir_name in dir_list:
        a_path = Path()
        a_path.header.frame_id = "map"
        with Reader( bags_folder+'/'+dir_name ) as reader:
            # topic and msgtype information is available on .connections list
            for connection in reader.connections:
                print(connection.topic, connection.msgtype)
        
            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/fixposition/odometry':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    a_pose = PoseStamped()
                    a_pose.pose.position.x = msg.pose.pose.position.x - ref_x
                    a_pose.pose.position.y = msg.pose.pose.position.y - ref_y
                    a_pose.pose.position.z = msg.pose.pose.position.z - ref_z
                    a_path.poses.append(a_pose)
        np_ecef_path_list.append(a_path)

    return np_ecef_path_list

def main(args=None):

    rclpy.init(args=args)

    node_ = rclpy.create_node('fp_ecef_plot')

    all_paths = readBags()

    publisher = node_.create_publisher(Path, 'fp_ecef_paths', 10)

    while rclpy.ok():
        for i in all_paths:
            publisher.publish(i)
        rclpy.spin_once(node_, timeout_sec=1.0)
    


if __name__ == '__main__':
    main()