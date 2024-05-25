#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class ObstacleFlagPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('obstacle_flag_publisher_node', anonymous=True)

        # Define publisher
        self.flag_pub = rospy.Publisher('/obstacle_flag', Bool, queue_size=10)

        # Timer to change the flag status
        self.flag = False
        self.timer = rospy.Timer(rospy.Duration(5), self.change_flag_status)  # Change every 5 seconds

    def change_flag_status(self, event):
        self.flag = not self.flag
        self.flag_pub.publish(Bool(self.flag))
        rospy.loginfo(f"Obstacle flag set to {self.flag}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_flag_publisher = ObstacleFlagPublisher()
        obstacle_flag_publisher.run()
    except rospy.ROSInterruptException:
        pass
