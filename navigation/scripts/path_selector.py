#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from turtlebot3_msgs.msg import wp_list 

class PathSelector:
    def __init__(self):
        # Initialize the node
        rospy.init_node('path_selector_node', anonymous=True)

        # Define subscribers
        self.global_path_sub = rospy.Subscriber('/global_path', wp_list, self.global_path_callback)
        self.local_path_sub = rospy.Subscriber('/local_path', wp_list, self.local_path_callback)
        self.obstacle_flag_sub = rospy.Subscriber('/obstacle_flag', Bool, self.obstacle_flag_callback)

        # Define publisher
        self.selected_path_pub = rospy.Publisher('/selected_path', Path, queue_size=10)
        self.visualization_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # Initialize path variables
        self.global_path = None
        self.local_path = None
        self.obstacle_flag = False

    def global_path_callback(self, msg):
        self.global_path = msg
        self.visualize_path(msg, "global_path")

    def local_path_callback(self, msg):
        self.local_path = msg
        self.visualize_path(msg, "local_path")

    def obstacle_flag_callback(self, msg):
        self.obstacle_flag = msg.data
        self.publish_selected_path()

    def publish_selected_path(self):
        if self.obstacle_flag and self.local_path:
            rospy.loginfo("Publishing local path")
            self.selected_path_pub.publish(self.local_path)
        elif self.global_path:
            rospy.loginfo("Publishing global path")
            self.selected_path_pub.publish(self.global_path)

    def visualize_path(self, path, path_type):
        marker = Marker()
        marker.header.frame_id = path.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = path_type
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0 if path_type == "global_path" else 0.0
        marker.color.g = 0.0 if path_type == "global_path" else 1.0
        marker.color.b = 0.0

        marker.points = [point.pose.position for point in path.poses]

        self.visualization_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        path_selector = PathSelector()
        path_selector.run()
    except rospy.ROSInterruptException:
        pass
