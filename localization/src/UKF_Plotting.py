#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt

# Global variables to store data
time_data_source1 = []
time_data_source2 = []
var1_data_source1 = []
var2_data_source1 = []
var1_data_source2 = []
var2_data_source2 = []

# Callback function for the first topic
def filter_callback(data):
    current_time = rospy.get_time()
    var1 = data.data[7]
    var2 = data.data[8]

    # Append new data to lists
    time_data_source1.append(current_time)
    var1_data_source1.append(var1)
    var2_data_source1.append(var2)

    # Debugging: print the received data
    rospy.loginfo(f"Filter callback - Time: {current_time}, Var1: {var1}, Var2: {var2}")

# Callback function for the second topic
def ground_truth_callback(data):
    current_time = rospy.get_time()
    var1 = data.pose[1].position.x
    var2 = data.pose[1].position.y

    # Append new data to lists
    time_data_source2.append(current_time)
    var1_data_source2.append(var1)
    var2_data_source2.append(var2)

    # Debugging: print the received data
    rospy.loginfo(f"Ground truth callback - Time: {current_time}, Var1: {var1}, Var2: {var2}")

# Plotting function for both filtered and ground truth data
def plot_data():
    plt.figure(figsize=(10, 8))

    plt.subplot(2, 2, 1)
    plt.plot(time_data_source1, var1_data_source1, label='Filtered X', color='blue')
    plt.xlabel('Time')
    plt.ylabel('X position')
    plt.title('Filtered X Position')
    plt.ylim(-5, 5)
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(time_data_source2, var1_data_source2, label='Ground Truth X', color='red')
    plt.xlabel('Time')
    plt.ylabel('X position')
    plt.title('Ground Truth X Position')
    plt.ylim(-5, 5)
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(time_data_source1, var2_data_source1, label='Filtered Y', color='blue')
    plt.xlabel('Time')
    plt.ylabel('Y position')
    plt.title('Filtered Y Position')
    plt.ylim(-5, 5)
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(time_data_source2, var2_data_source2, label='Ground Truth Y', color='red')
    plt.xlabel('Time')
    plt.ylabel('Y position')
    plt.title('Ground Truth Y Position')
    plt.ylim(-5, 5)
    plt.legend()

    plt.tight_layout()

# Main function
if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('UKF_plotting', anonymous=True)

        # Subscribers for both topics
        rospy.Subscriber("/filtered_state", Float64MultiArray, filter_callback)  # Adjust the topic and message type
        rospy.Subscriber("/gazebo/model_states", ModelStates, ground_truth_callback)  # Adjust the topic and message type

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")

    # Plot the data after stopping the node
    plot_data()
    plt.show()
