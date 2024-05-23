#!/usr/bin/env python3
# import rospy
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt
# from turtlebot3_msgs.msg import wp_list

# class TebWaypointsExtractor:
#     def __init__(self):
#         rospy.init_node('teb_waypoints_extractor', anonymous=True)
#         rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.path_callback)
#         self.pub = rospy.Publisher('/local_path', wp_list, queue_size=10)
#         self.waypoints = []

#     def path_callback(self, path_msg):
#         self.waypoints = self.extract_waypoints(path_msg)
#         rospy.loginfo("Waypoints: {}".format(self.waypoints))
#         self.plot_waypoints()

#     def extract_waypoints(self, path_msg):
#         waypoints = []
#         for pose_stamped in path_msg.poses:
#             x = pose_stamped.pose.position.x
#             y = pose_stamped.pose.position.y
#             waypoints.append((x, y))
#         self.pub.publish(self.waypoints)     
#         return waypoints

#     def plot_waypoints(self):
#         if not self.waypoints:
#             return
        
#         x_coords, y_coords = zip(*self.waypoints)
        
#         plt.figure()
#         plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b')
#         plt.title('TebLocalPlannerROS Waypoints')
#         plt.xlabel('X')
#         plt.ylabel('Y')
#         plt.grid(True)
#         plt.show()

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     extractor = TebWaypointsExtractor()
#     extractor.run()

#############################################################################################################################

import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

class TebWaypointsExtractor:
    def __init__(self):
        rospy.init_node('teb_waypoints_extractor', anonymous=True)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.path_callback)
        self.waypoints = []

    def path_callback(self, path_msg):
        self.waypoints = self.extract_waypoints(path_msg)
        rospy.loginfo("Waypoints: {}".format(self.waypoints))
        self.plot_waypoints()

    def extract_waypoints(self, path_msg):
        waypoints = []
        for pose_stamped in path_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            waypoints.append((x, y))
        return waypoints

    def plot_waypoints(self):
        if not self.waypoints:
            return

        x_coords, y_coords = zip(*self.waypoints)
        
        plt.figure()
        plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b')
        plt.title('TebLocalPlannerROS Waypoints')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.show()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    extractor = TebWaypointsExtractor()
    extractor.run()
