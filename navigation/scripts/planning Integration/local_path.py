#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class LocalPathPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('local_path_publisher_node', anonymous=True)

        # Define publisher
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=10)

        # Logical waypoints for the local path (list of tuples)
        self.original_waypoints = [
            (3.5, 3.5),
            (3.75, 3.75),
            (4.0, 4.0),
            (4.0, 4.5),
            (4.0, 5.0),
            (4.0, 5.5),
            (4.0, 6.0),
            (4.0, 6.5),
            (4.0, 7.0),
            (4.0, 7.5),
            (4.0, 8.0),
            (4.0, 8.5),
            (4.0, 9.5),
            (6.0, 8.5),
            (8.0, 6.0)
        ]

        # Generate interpolated waypoints
        self.waypoints = self.interpolate_waypoints(self.original_waypoints, max_distance=0.25)

        # Set a timer to publish the path at regular intervals
        self.publish_interval = rospy.Duration(1)  # 1 second
        self.timer = rospy.Timer(self.publish_interval, self.publish_path)

    def interpolate_waypoints(self, waypoints, max_distance):
        interpolated_points = []

        for i in range(len(waypoints) - 1):
            start_point = waypoints[i]
            end_point = waypoints[i + 1]

            distance = math.sqrt((end_point[0] - start_point[0]) ** 2 + (end_point[1] - start_point[1]) ** 2)
            num_points = int(math.ceil(distance / max_distance))

            for j in range(num_points):
                t = j / num_points
                interpolated_x = start_point[0] + t * (end_point[0] - start_point[0])
                interpolated_y = start_point[1] + t * (end_point[1] - start_point[1])
                interpolated_points.append((interpolated_x, interpolated_y))

        interpolated_points.append(waypoints[-1])  # Add the last waypoint
        return interpolated_points

    def publish_path(self, event):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = path.header.frame_id
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.path_pub.publish(path)
        rospy.loginfo("Published local path with {} points".format(len(self.waypoints)))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        local_path_publisher = LocalPathPublisher()
        local_path_publisher.run()
    except rospy.ROSInterruptException:
        pass
