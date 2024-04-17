#!/usr/bin/env python3

import rospy
from turtlebot3_msgs.msg import wp_list  # Import your custom message

# File path
file_path ="/home/seif/GP_ws/src/ROVER_HighLevel/planning/global_path_planning/scripts/data.txt"



def tuple_list_publisher():
    rospy.init_node('tuple_list_publisher', anonymous=True)
    pub = rospy.Publisher('tuple_list_topic', wp_list, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        x_way_points_list=[]
        y_way_points_list=[]
        with open(file_path, 'r') as file:
        # Iterate over each line in the file
            for line in file:
        # Strip whitespace and newline characters from the line, then split by whitespace
                elements = line.strip().split('\n')

        # Iterate over each line
                for line in elements:
        # Remove square brackets and split by comma
                    elements = line.strip()[1:-1].split(',')
        # Extract the first two elements
                    x = float(elements[0])
                    y = float(elements[1])
                    x_way_points_list.append(x)
                    y_way_points_list.append(y)

        # Create a message
            msg = wp_list()
            msg.length = len(x_way_points_list)
        # Populate tuple data
            msg.a = x_way_points_list
            msg.b = y_way_points_list

        # Publish the message
            pub.publish(msg)
            rospy.loginfo("Published a list of tuples")

            rate.sleep()

if __name__ == '__main__':
    try:
        tuple_list_publisher()
    except rospy.ROSInterruptException:
        pass
