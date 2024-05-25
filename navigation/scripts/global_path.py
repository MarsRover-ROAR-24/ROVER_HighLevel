#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class GlobalPathPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('global_path_publisher_node', anonymous=True)

        # Define publisher
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)

        # Waypoints for the global path (list of tuples)
        self.waypoints = [(x, x) for x in range(11)]  # (0, 0) to (10, 10)

        # Set a timer to publish the path at regular intervals
        self.publish_interval = rospy.Duration(1)  # 1 second
        self.timer = rospy.Timer(self.publish_interval, self.publish_path)

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
        rospy.loginfo("Published global path")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        global_path_publisher = GlobalPathPublisher()
        global_path_publisher.run()
    except rospy.ROSInterruptException:
        pass
