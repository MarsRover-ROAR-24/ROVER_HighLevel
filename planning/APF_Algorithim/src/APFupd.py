#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

def map_callback(data):
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    origin = data.info.origin
    grid_data = np.array(data.data).reshape((height, width))

    # Convert grid to binary image
    binary_grid = np.uint8((grid_data == 100) * 255)

    # Find contours of the obstacles
    contours, _ = cv2.findContours(binary_grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    obstacle_centers = []

    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            obstacle_centers.append((cx, cy))

    # Convert to Gazebo coordinates
    gazebo_coordinates = []
    for (cx, cy) in obstacle_centers:
        gx = origin.position.x + cx * resolution
        gy = origin.position.y + cy * resolution
        gazebo_coordinates.append(Point(gx, gy, 0))

    # Print the coordinates
    for coord in gazebo_coordinates:
        rospy.loginfo(f"Obstacle center at Gazebo coordinates: ({coord.x}, {coord.y})")

def main():
    rospy.init_node('obstacle_center_extractor', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
