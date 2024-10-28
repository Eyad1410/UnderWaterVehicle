#! /usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ConvertENU
from nav_msgs.msg import Path
import os
import csv
from ament_index_python.packages import get_package_share_directory

def convert_enu(node_, waypoints_file, force_orientation):

    ecef_req = ConvertENU.Request()
    ecef_req.ecef_path.header.frame_id = "map"
    jump_cnt = 0
    with open(waypoints_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        last_pose = PoseStamped()
        for row in csv_reader:
            a_pose = PoseStamped()
            a_pose.header.frame_id = "map"
            a_pose.pose.position.x = float(row[0])
            a_pose.pose.position.y = float(row[1])
            a_pose.pose.position.z = float(row[2])
            if(force_orientation):
                a_pose.pose.orientation.x = 0.0
                a_pose.pose.orientation.y = 0.0
                a_pose.pose.orientation.z = 0.0
                a_pose.pose.orientation.w = 1.0
            else:
                a_pose.pose.orientation.x = float(row[3])
                a_pose.pose.orientation.y = float(row[4])
                a_pose.pose.orientation.z = float(row[5])
                a_pose.pose.orientation.w = float(row[6])                
            ecef_req.ecef_path.poses.append(a_pose)

        

    #node_.get_logger().info(ecef_req.ecef_path)
    node_.get_logger().info("Start calling service")
    srv_client = node_.create_client(ConvertENU, 'ecef2enu_conversion')
    while not srv_client.wait_for_service(timeout_sec=1.0):
        node_.get_logger().info('service not available, waiting again...')
        rclpy.spin_once(node_, timeout_sec=1.0)
    future = srv_client.call_async(ecef_req)
    rclpy.spin_until_future_complete(node_, future)
    #node_.get_logger().info(future.result().enu_path.poses)
    
    path = Path()
    path.header.frame_id = "map"
    for i in future.result().enu_path.poses:
        path.poses.append(i)
    
    return path

def raw_ecef(node_, waypoints_file):

    path = Path()
    path.header.frame_id = "map"

    with open(waypoints_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        last_pose = PoseStamped()
        for row in csv_reader:
            a_pose = PoseStamped()
            a_pose.header.frame_id = "map"
            a_pose.pose.position.x = float(row[0])-3288218
            a_pose.pose.position.y = float(row[1])-4745916
            a_pose.pose.position.z = float(row[2])-2700936
            a_pose.pose.orientation.x = float(row[3])
            a_pose.pose.orientation.y = float(row[4])
            a_pose.pose.orientation.z = float(row[5])
            a_pose.pose.orientation.w = float(row[6])                
            path.poses.append(a_pose)
    
    return path

def main(args=None):

    rclpy.init(args=args)

    node_ = rclpy.create_node('ros2_manager')

    all_paths = []

    
    pkg_dir = get_package_share_directory('enu_transform')
    waypoints_file = os.path.join(pkg_dir, 'data', 'waypoints_new.csv')
    path1 = convert_enu(node_, waypoints_file, False)
    all_paths.append(path1)

    waypoints_file = os.path.join(pkg_dir, 'data', 'waypoints_new2.csv')
    path2 = convert_enu(node_, waypoints_file, False)
    all_paths.append(path2)

    '''
    pkg_dir = get_package_share_directory('enu_transform')
    waypoints_file = os.path.join(pkg_dir, 'data', 'waypoints.csv')
    path1 = raw_ecef(node_, waypoints_file)
    all_paths.append(path1)

    pkg_dir = get_package_share_directory('enu_transform')
    waypoints_file = os.path.join(pkg_dir, 'data', 'waypoints_new.csv')
    path2 = raw_ecef(node_, waypoints_file)
    all_paths.append(path2)

    waypoints_file = os.path.join(pkg_dir, 'data', 'waypoints_new2.csv')
    path3 = raw_ecef(node_, waypoints_file)
    all_paths.append(path3)
    '''
    publisher = node_.create_publisher(Path, 'enu_path', 10)

    while rclpy.ok():
        for i in all_paths:
            publisher.publish(i)
        rclpy.spin_once(node_, timeout_sec=1.0)
    


if __name__ == '__main__':
    main()