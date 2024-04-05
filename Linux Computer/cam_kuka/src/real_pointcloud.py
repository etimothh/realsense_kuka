#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import pyrealsense2 as rs
import numpy as np

"""
RealSense Point Cloud Publisher

This program captures depth and color frames from a RealSense D455 camera and generates a point cloud from the depth data. 
The point cloud is then published to the 'pcl_realsense' topic in the ROS 2 ecosystem. This allows other ROS 2 nodes to 
subscribe to the point cloud data for further processing or visualization.

Author: [Timothé Kobak]
Date: [Thursday, 4 April]
"""

class RealSensePointCloudPublisher(Node):
    def __init__(self):
        super().__init__('realsense_pointcloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'pcl_realsense', 10)

    def generate_point_cloud(self):
        # Create a pipeline
        pipeline = rs.pipeline()

        # Create a config object
        config = rs.config()

        # Enable depth and color streams
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start the pipeline
        pipeline.start(config)

        try:
            print("Generating point cloud...")
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # Generate point cloud
                pc = rs.pointcloud()
                points = pc.calculate(depth_frame)
                vtx = np.asanyarray(points.get_vertices())
                tex = np.asanyarray(points.get_texture_coordinates())

                # Clean up the point cloud
                vtx = vtx[tex[:, 0] > 0]
                vtx = vtx[tex[:, 1] > 0]

                # Publish the point cloud
                msg = PointCloud2()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'  # Replace 'camera_link' with your frame ID
                msg.height = 1
                msg.width = len(vtx)
                msg.fields.append(PointField(
                    name="x", offset=0, datatype=PointField.FLOAT32, count=1))
                msg.fields.append(PointField(
                    name="y", offset=4, datatype=PointField.FLOAT32, count=1))
                msg.fields.append(PointField(
                    name="z", offset=8, datatype=PointField.FLOAT32, count=1))
                msg.point_step = 12
                msg.row_step = msg.point_step * len(vtx)
                msg.is_bigendian = False
                msg.is_dense = True
                msg.data = vtx.tostring()

                self.publisher_.publish(msg)
                print("Point cloud published")

        except Exception as e:
            self.get_logger().error('An error occurred: %s' % str(e))

    def run(self):
        self.generate_point_cloud()

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePointCloudPublisher()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
