#! /usr/bin/env python3

import rospy
from turtlebot3_msgs.msg import wp_list

# File path
file_path = "/home/shams/turtlebot_ws/src/global_path_planning/scripts/data.txt"
way_points_list=[]


def publisher():
    # Initialize ROS node
    rospy.init_node('custom_publisher', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('way_points', wp_list, queue_size=10)

    # Rate at which to publish the message
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():

      with open(file_path, 'r') as file:
        # Iterate over each line in the file
        for line in file:
        # Strip whitespace and newline characters from the line, then split by whitespace
          elements = line.strip().split('\n')

        # Iterate over each line
          for line in elements:
        # Create a message object

        # Remove square brackets and split by comma
            elements = line.strip()[1:-1].split(',')
        # Extract the first two elements
            x = float(elements[0])
            y = float(elements[1])
            way_points_list.append((x,y))


        msg = wp_list()
        msg.waypoints = way_points_list
        msg.goalpointx = way_points_list[0][0]
        msg.goalpointy = way_points_list[0][1]

          # Publish the message
        pub.publish(msg)
        rospy.loginfo(f"Published waypoints message: {msg.waypoints}")
        rospy.loginfo(f"Published Goalpoints: {msg.goalpointx,msg.goalpointy}")
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass