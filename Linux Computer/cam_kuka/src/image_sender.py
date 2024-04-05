#!/usr/bin/python3

"""
Triggered RGB Image Sender

This program captures a colored image from a camera (e.g., RealSense D455) and sends it over TCP/IP upon receiving a trigger command.
It subscribes to the 'image_raw' topic to receive the latest colored image data and waits for a trigger command over TCP/IP to send the image.

Author: [Timothé Kobak]
Date: [Current Date]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import socket
import struct
import threading

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')
        self.subscriber_ = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.latest_image = None
        self.lock = threading.Lock()
        self.send_trigger_thread = threading.Thread(target=self.send_trigger)
        self.send_trigger_thread.daemon = True
        self.send_trigger_thread.start()

    def image_callback(self, msg):
        with self.lock:
            self.latest_image = msg

    def send_trigger(self):
        host = '127.0.0.1'
        port = 12345

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((host, port))
        s.listen(1)

        print("Trigger server listening on {}:{}".format(host, port))

        while True:
            conn, addr = s.accept()
            print("Connected by", addr)
            data = conn.recv(1024).decode()

            if data == 'trigger':
                with self.lock:
                    if self.latest_image is not None:
                        image_data = self.latest_image
                        self.send_image(image_data)

            conn.close()

    def send_image(self, msg):
        # Convert the image message to a numpy array
        image_np = np.array(msg.data).reshape((msg.height, msg.width, 3))

        # Convert the numpy array to bytes
        image_bytes = image_np.tobytes()

        # Send the image data over TCP/IP
        host = '127.0.0.1'  # Update with the destination IP address
        port = 54321  # Update with the destination port number

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall(struct.pack('>I', len(image_bytes)) + image_bytes)
        s.close()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
