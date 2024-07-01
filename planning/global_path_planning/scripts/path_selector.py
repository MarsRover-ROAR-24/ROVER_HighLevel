#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker

class PathSelector:
    def __init__(self):
        # Initialize the node
        rospy.init_node('path_selector_node', anonymous=True)

        # Define subscribers
        self.global_path_sub = rospy.Subscriber('/global_path', Path, self.global_path_callback)
        self.local_path_sub = rospy.Subscriber('/local_path', Path, self.local_path_callback)
        self.obstacle_flag_sub = rospy.Subscriber('/obstacle_flag', Bool, self.obstacle_flag_callback)
        self.local_cm_subscriber = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.publish_selected_path)  

        # Define publisher
        self.selected_path_pub = rospy.Publisher('/selected_path', Path, queue_size=10)
        
        # Initialize path variables
        self.global_path = None
        self.local_path = None
        self.obstacle_flag = False

        self.rate = rospy.Rate(10)

    def global_path_callback(self, msg):
        self.global_path = msg
        self.visualize_path(msg, "global_path")

    def local_path_callback(self, msg):
        self.local_path = msg
        self.visualize_path(msg, "local_path")

    def local_costmap_callback(self, msg):
        self.check_for_obstacles(msg)

    def obstacle_flag_callback(self, msg):
        self.obstacle_flag = msg.data
        self.publish_selected_path()


    def check_for_obstacles(self, costmap):
        has_obstacle = False
        for value in costmap.data:
            if value > 0:
                has_obstacle = True
                break

        self.obstacle_flag = has_obstacle
        rospy.loginfo("Obstacle flag set to: {}".format(self.obstacle_flag))
        self.publish_selected_path()


    def publish_selected_path(self):
        if self.obstacle_flag and self.local_path:
            rospy.loginfo("Publishing local path")
            self.selected_path_pub.publish(self.local_path)
        elif self.global_path:
            rospy.loginfo("Publishing global path")
            self.selected_path_pub.publish(self.global_path)


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            
if __name__ == '__main__':
    try:
        path_selector = PathSelector()
        path_selector.run()
    except rospy.ROSInterruptException:
        pass
