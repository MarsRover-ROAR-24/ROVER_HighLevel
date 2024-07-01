#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2
import numpy as np

def point_cloud_callback(msg):
    # Convert PointCloud2 to numpy array
    pc = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc = np.array(list(pc))

    # Calculate ranges and intensities for LaserScan
    ranges = np.linalg.norm(pc, axis=1)
    intensities = np.zeros(len(ranges))

    # Create LaserScan message
    scan_msg = LaserScan()
    scan_msg.header = msg.header
    scan_msg.angle_min = -np.pi / 2
    scan_msg.angle_max = np.pi / 2
    scan_msg.angle_increment = np.pi / len(ranges)
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.1
    scan_msg.range_min = 0.0
    scan_msg.range_max = np.max(ranges)
    scan_msg.ranges = ranges.tolist()
    scan_msg.intensities = intensities.tolist()

    # Publish LaserScan message
    pub.publish(scan_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_to_laserscan')

    # Subscriber for the point cloud
    rospy.Subscriber("/3d_image/3d_cloud", PointCloud2, point_cloud_callback)

    # Publisher for the LaserScan data
    pub = rospy.Publisher("/kinect/scan", LaserScan, queue_size=10)

    rospy.spin()
