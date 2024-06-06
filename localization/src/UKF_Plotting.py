#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
from collections import deque

# Global variables to store data
buffer_size = 1000  # Adjust based on your needs
time_buffer_source1 = deque(maxlen=buffer_size)
time_buffer_source2 = deque(maxlen=buffer_size)
var1_buffer_source1 = deque(maxlen=buffer_size)
var2_buffer_source1 = deque(maxlen=buffer_size)
var1_buffer_source2 = deque(maxlen=buffer_size)
var2_buffer_source2 = deque(maxlen=buffer_size)

# Matched data for plotting
matched_time = []
matched_var1_source1 = []
matched_var2_source1 = []
matched_var1_source2 = []
matched_var2_source2 = []

# Callback function for the first topic
def filter_callback(data):
    current_time = rospy.get_time()
    var1 = data.data[7]
    var2 = data.data[8]

    # Append new data to buffer
    time_buffer_source1.append(current_time)
    var1_buffer_source1.append(var1)
    var2_buffer_source1.append(var2)

    # Debugging: print the received data
    # rospy.loginfo(f"Filter callback - Time: {current_time}, Var1: {var1}, Var2: {var2}")

    # Try to find a matching timestamp
    match_data()

# Callback function for the second topic
def ground_truth_callback(data):
    current_time = rospy.get_time()
    var1 = data.pose[1].position.x
    var2 = data.pose[1].position.y

    # Append new data to buffer
    time_buffer_source2.append(current_time)
    var1_buffer_source2.append(var1)
    var2_buffer_source2.append(var2)

    # Debugging: print the received data
    # rospy.loginfo(f"Ground truth callback - Time: {current_time}, Var1: {var1}, Var2: {var2}")

    # Try to find a matching timestamp
    match_data()

# Match data based on timestamps
def match_data():
    if len(time_buffer_source1) > 0 and len(time_buffer_source2) > 0:
        # Find the closest matching timestamp
        for t1, t2 in zip(time_buffer_source1, time_buffer_source2):
            if abs(t1 - t2) < 0.01:  # Adjust threshold as needed
                matched_time.append(t1)
                matched_var1_source1.append(var1_buffer_source1[time_buffer_source1.index(t1)])
                matched_var2_source1.append(var2_buffer_source1[time_buffer_source1.index(t1)])
                matched_var1_source2.append(var1_buffer_source2[time_buffer_source2.index(t2)])
                matched_var2_source2.append(var2_buffer_source2[time_buffer_source2.index(t2)])
                break

# Plotting function for both filtered and ground truth data
def plot_data():
    plt.figure(figsize=(10, 8))

    # Plotting the filtered states
    plt.subplot(2, 1, 1)
    plt.plot(matched_var1_source1, matched_var2_source1, label='Filtered State', color='blue')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Filtered State Position')
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.legend()

    # Plotting the ground truth states
    plt.subplot(2, 1, 2)
    plt.plot(matched_var1_source2, matched_var2_source2, label='Ground Truth State', color='red')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Ground Truth State Position')
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
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
