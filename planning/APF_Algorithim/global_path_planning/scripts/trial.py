#!/usr/bin/env python3

import rospy
from turtlebot3_msgs.msg import wp_list   # Import your custom message


def tuple_list_callback(msg):
    received_tuples = []

    rospy.loginfo("Received a list of tuples:")

    for i in range(msg.length):
        tuple_data = (msg.a[i], msg.b[i])
        received_tuples.append(tuple_data)
    rospy.loginfo(received_tuples)

def tuple_list_subscriber():
    rospy.init_node('tuple_list_subscriber', anonymous=True)
    rospy.Subscriber('tuple_list_topic', wp_list, tuple_list_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        tuple_list_subscriber()
    except rospy.ROSInterruptException:
        pass
