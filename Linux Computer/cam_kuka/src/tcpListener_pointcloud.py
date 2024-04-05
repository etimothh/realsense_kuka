#!/usr/bin/python3
"""
RealSense Point Cloud Sender

This program subscribes to the 'pcl_realsense' topic to receive point cloud data from a RealSense camera. 
It also listens for trigger commands over TCP/IP to initiate the sending process. Upon receiving a trigger command, 
the latest point cloud data is serialized and sent over TCP/IP to another destination for further processing.

Author: [Timothé Kobak]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pyrealsense2 as rs
import numpy as np
import socket
import struct
import threading

class PointCloudSender(Node):
    def __init__(self):
        super().__init__('point_cloud_sender')
        self.subscriber_ = self.create_subscription(PointCloud2, 'pcl_realsense', self.point_cloud_callback, 10)
        self.lock = threading.Lock()
        self.send_trigger_thread = threading.Thread(target=self.send_trigger)
        self.send_trigger_thread.daemon = True
        self.send_trigger_thread.start()

    def point_cloud_callback(self, msg):
        with self.lock:
            self.latest_point_cloud = msg

    def send_trigger(self):
        host = '127.0.0.1'
        port = 12345

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((host, port))
        s.listen(1)

        print("Trigger server listening on {}:{}".format(host, port))

        while True:
            print("Waiting for trigger command...")
            conn, addr = s.accept()
            print("Connected by", addr)
            data = conn.recv(1024).decode()

            if data == 'trigger':
                with self.lock:
                    if hasattr(self, 'latest_point_cloud'):
                        point_cloud_data = self.latest_point_cloud
                        print("Trigger received. Sending point cloud...")
                        self.send_point_cloud(point_cloud_data)

            conn.close()

    def send_point_cloud(self, msg):
        # Convert the point cloud message to bytes
        serialized_data = msg.serialize()

        # Send the serialized point cloud data over TCP/IP
        host = '127.0.0.1'  # Update with the destination IP address
        port = 54321  # Update with the destination port number

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall(struct.pack('>I', len(serialized_data)) + serialized_data)
        s.close()
        print("Point cloud sent to {}:{}".format(host, port))

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
