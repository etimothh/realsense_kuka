import open3d as o3d
import numpy as np
import pyrealsense2 as rs

# Function to capture a point cloud at a given point using RealSense D455
def capture_point_cloud(max_depth=0.8):
    input("Move to the desired point and press Enter to capture the point cloud.")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    pipeline.stop()

    pc = rs.pointcloud()
    points = pc.calculate(depth_frame)
    vertices = np.asanyarray(points.get_vertices())
    points_array = np.array(vertices.tolist())

    # Filter points that are within the specified max depth
    max_depth_indices = points_array[:, 2] <= max_depth
    valid_points_array = points_array[max_depth_indices]

    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(valid_points_array))

# Given camera poses (in this case, just placeholders)
points_2d = [(-7.3, -359.9), (-10.4, -651.1), (250, -651.1), (411.8, -402.8)]

# Capture point clouds at each specified point
point_clouds = [capture_point_cloud(max_depth=0.8) for _ in range(len(points_2d))]

# Function to merge multiple point clouds considering estimated camera poses
def merge_point_clouds(point_clouds, points_2d):
    # Create an empty point cloud to store the merged point cloud
    merged_point_cloud = o3d.geometry.PointCloud()

    # Extracting coordinates of P1, P2, P3, P4
    p1 = points_2d[0]
    p2 = points_2d[1]
    p3 = points_2d[2]
    p4 = points_2d[3]

    # Translate each point cloud to the respective estimated camera pose
    for i in range(len(point_clouds)):
        # Translate each point cloud to the respective estimated camera pose
        if i == 0:  # for the first point cloud
            translation = np.array([0, 0, 0])
        elif i == 1:  # for the second point cloud
            translation = np.array([ (p2[1]-p1[1])    ,p2[0]-p1[0] , 0]) 
        elif i == 2:  # for the third point cloud
            translation = np.array([(p3[1]-p1[1]), (p3[0]-p1[0]),0])
        elif i == 3:  # for the fourth point cloud
            translation = np.array([(p4[1]-p1[1]), (p4[0]-p1[0]), 0])
        else:
            raise ValueError("More points than expected.")
        
        # Translate point cloud to the estimated camera pose
        #for i, point in enumerate(point_clouds[i].points):
            #rint(f"Point {i+1}: {point}")
        point_clouds[i].points = o3d.utility.Vector3dVector(np.asarray((point_clouds[i].points))+ translation/1000)
        
        # Add the translated point cloud to the merged point cloud
        merged_point_cloud += point_clouds[i]

    return merged_point_cloud

# Merge point clouds considering estimated camera poses
merged_point_cloud = merge_point_clouds(point_clouds, points_2d)

# Print each point of the merged point cloud


# Visualize the merged point cloud
o3d.visualization.draw_geometries([merged_point_cloud], window_name="Merged Point Cloud")