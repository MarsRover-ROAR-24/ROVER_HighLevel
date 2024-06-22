#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import threading

class ObstacleFlagPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('obstacle_flag_publisher_node', anonymous=True)

        # Define publisher
        self.flag_pub = rospy.Publisher('/obstacle_flag', Bool, queue_size=10)

        # Initial flag status
        self.flag = False

        # Start a thread to listen for user input
        self.input_thread = threading.Thread(target=self.listen_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def listen_for_input(self):
        while not rospy.is_shutdown():
            user_input = input("Enter 'on' to set flag to True, 'off' to set flag to False: ")
            if user_input.lower() == 'on':
                self.flag = True
            elif user_input.lower() == 'off':
                self.flag = False
            else:
                rospy.logwarn("Invalid input. Enter 'on' or 'off'.")
                continue

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
