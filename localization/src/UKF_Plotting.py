#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

# Global variables to store data
time_data = []
var1_data_source1 = []
var2_data_source1 = []
var1_data_source2 = []
var2_data_source2 = []

# Mutex for thread safety
data_lock = threading.Lock()

# Callback function for the first topic
def filter_callback(data):
    current_time = rospy.get_time()
    var1 = data.data[7]
    var2 = data.data[8]

    # Append new data to lists with thread safety
    with data_lock:
        time_data.append(current_time)
        var1_data_source1.append(var1)
        var2_data_source1.append(var2)

    # Debugging: print the received data
    rospy.loginfo(f"Filter callback - Time: {current_time}, Var1: {var1}, Var2: {var2}")

# Callback function for the second topic
def ground_truth_callback(data):
    current_time = rospy.get_time()
    var1 = data.pose[1].position.y
    var2 = data.pose[1].position.x

    # Append new data to lists with thread safety
    with data_lock:
        var1_data_source2.append(var1)
        var2_data_source2.append(var2)

    # Debugging: print the received data
    rospy.loginfo(f"Ground truth callback - Time: {current_time}, Var1: {var1}, Var2: {var2}")

# Plotting function
def animate(i, time_data, var1_data_source1, var2_data_source1, var1_data_source2, var2_data_source2):
    # Acquire data lock for thread safety
    with data_lock:
        ax1.clear()
        ax2.clear()

        # Plot source 1 data
        if len(time_data) == len(var1_data_source1):
            ax1.plot(time_data, var1_data_source1, label='Filtered X', color='blue')
        if len(time_data) == len(var2_data_source1):
            ax2.plot(time_data, var2_data_source1, label='Filtered Y', color='blue')

        # Plot source 2 data
        if len(time_data) == len(var1_data_source2):
            ax1.plot(time_data, var1_data_source2, label='Ground Truth X', color='red')
        if len(time_data) == len(var2_data_source2):
            ax2.plot(time_data, var2_data_source2, label='Ground Truth Y', color='red')

        ax1.set_xlabel('Time')
        ax1.set_ylabel('X position')
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Y position')

        ax1.legend()
        ax2.legend()

        # Debugging: print data lengths
        rospy.loginfo(f"Time data length: {len(time_data)}, Var1 source 1 length: {len(var1_data_source1)}, Var1 source 2 length: {len(var1_data_source2)}")
        rospy.loginfo(f"Var2 source 1 length: {len(var2_data_source1)}, Var2 source 2 length: {len(var2_data_source2)}")

# Main function
if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('UKF_plotting', anonymous=True)

        # Subscribers for both topics
        rospy.Subscriber("/filtered_state", Float64MultiArray, filter_callback)  # Adjust the topic and message type
        rospy.Subscriber("/gazebo/model_states", ModelStates, ground_truth_callback)  # Adjust the topic and message type

        # Set up the plot
        fig, (ax1, ax2) = plt.subplots(2, 1)

        # Start the animation
        ani = animation.FuncAnimation(fig, animate, fargs=(time_data, var1_data_source1, var2_data_source1, var1_data_source2, var2_data_source2), interval=1000)
        plt.show()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")
