#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import os

data = []


# Callback function to handle received Pose messages
def pose_callback(msg):
    # Extract position coordinates from the received message
    x = msg.position.x
    y = msg.position.y
    points = [x,y]
    data.append(points)

        # Specify the file name
    file_name = "/home/shams/turtlebot_ws/src/APF_Algorithim/src/sample_data.txt"
        # Open the file in write mode (use 'w' for writing)
    with open(file_name, 'a') as file:
        # Write the data to the file
        file.write(str(points) + "\n")

    if round(x,1) == 0.5 and round(y,1) == 4.5:
        rospy.loginfo("bye!")
    

def main():
    # Initialize the ROS node
    rospy.init_node('pose_subscriber')

    # Subscribe to the /APF_Des_Pos topic with a callback function
    rospy.Subscriber('/APF_Des_Pos', Pose, pose_callback)

    # Spin to keep the node alive until it's stopped manually
    rospy.spin()

if __name__ == '__main__':

    main()

