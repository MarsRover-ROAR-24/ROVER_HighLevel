#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt
# from turtlebot3_msgs.msg import wp_list

class Control:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.velocitylf_publisher = rospy.Publisher('/wheel_lhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocityrf_publisher = rospy.Publisher('/wheel_rhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocitylr_publisher = rospy.Publisher('/wheel_lhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocityrr_publisher = rospy.Publisher('/wheel_rhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocitylm_publisher = rospy.Publisher('/wheel_lhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.velocityrm_publisher = rospy.Publisher('/wheel_rhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/filtered_state', Float64MultiArray, self.update_pose)
        self.yaw_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_yaw)

        # self.path_subscriber = rospy.Subscriber('tuple_list_topic', wp_list, self.tuple_list_callback)  #===>

        self.waypoints= [
                (0, 0),
                (0.1, 0.05),
                (0.2, 0.1),
                (0.3, 0.15),
                (0.4, 0.2),
                (0.5, 0.25),
                (0.5, 1),
                (0.6, 1.2),
                (0.7, 1.4),
                (0.8, 1.6),
                (0.9, 1.8),
                (1, 2),
                (1.05, 2.05),
                (1.1, 2.1),
                (1.15, 2.15),
                (1.2, 2.2),
                (1.25, 2.25),
                (1.3, 2.3),
                (1.35, 2.35),
                (1.4, 2.4),
                (1.45, 2.45),
                (1.5, 2.5),
                (1.55, 2.55),
                (1.6, 2.6),
                (1.65, 2.65),
                (1.7, 2.7),
                (1.75, 2.75),
                (1.8, 2.8),
                (1.85, 2.85),
                (1.9, 2.9),
                (1.95, 2.95),
                (2, 3),
                (2.05, 3.05),
                (2.1, 3.1),
                (2.15, 3.15),
                (2.2, 3.2),
                (2.25, 3.25),
                (2.3, 3.3),
                (2.35, 3.35),
                (2.4, 3.4),
                (2.45, 3.45),
                (2.5, 3.5),
                (2.55, 3.55),
                (2.6, 3.6),
                (2.65, 3.65),
                (2.7, 3.7),
                (2.75, 3.75),
                (2.8, 3.8),
                (2.85, 3.85),
                (2.9, 3.9),
                (2.95, 3.95),
                (3, 4),
                (3.05, 4.05),
                (3.1, 4.1),
                (3.15, 4.15),
                (3.2, 4.2),
                (3.25, 4.25),
                (3.3, 4.3),
                (3.35, 4.35),
                (3.4, 4.4),
                (3.45, 4.45),
                (3.5, 4.5),
                (3.55, 4.55),
                (3.6, 4.6),
                (3.65, 4.65),
                (3.7, 4.7),
                (3.75, 4.75),
                (3.8, 4.8),
                (3.85, 4.85),
                (3.9, 4.9),
                (3.95, 4.95),
                (4, 5),
                (4.05, 5.05),
                (4.1, 5.1),
                (4.15, 5.15),
                (4.2, 5.2),
                (4.25, 5.25),
                (4.3, 5.3),
                (4.35, 5.35),
                (4.4, 5.4),
                (4.45, 5.45),
                (4.5, 5.5),
                (4.55, 5.55),
                (4.6, 5.6),
                (4.65, 5.65),
                (4.7, 5.7),
                (4.75, 5.75),
                (4.8, 5.8),
                (4.85, 5.85),
                (4.9, 5.9),
                (4.95, 5.95),
                (5, 6),
                (5.05, 6.05),
                (5.1, 6.1),
                (5.15, 6.15),
                (5.2, 6.2),
                (5.25, 6.25),
                (5.3, 6.3),
                (5.35, 6.35),
                (5.4, 6.4),
                (5.45, 6.45),
                (5.5, 6.5),
                (5.55, 6.55),
                (5.6, 6.6),
                (5.65, 6.65),
                (5.7, 6.7),
                (5.75, 6.75),
                (5.8, 6.8),
                (5.85, 6.85),
                (5.9, 6.9),
                (5.95, 6.95),
                (6, 7),
                (6.05, 7.05),
                (6.1, 7.1),
                (6.15, 7.15),
                (6.2, 7.2),
                (6.25, 7.25),
                (6.3, 7.3),
                (6.35, 7.35),
                (6.4, 7.4),
                (6.45, 7.45),
                (6.5, 7.5),
                (6.55, 7.55),
                (6.6, 7.6),
                (6.65, 7.65),
                (6.7, 7.7),
                (6.75, 7.75),
                (6.8, 7.8),
                (6.85, 7.85),
                (6.9, 7.9),
                (6.95, 7.95),
                (7, 8),
                (7.05, 8.05),
                (7.1, 8.1),
                (7.15, 8.15),
                (7.2, 8.2),
                (7.25, 8.25),
                (7.3, 8.3),
                (7.35, 8.35),
                (7.4, 8.4),
                (7.45, 8.45),
                (7.5, 8.5),
                (7.55, 8.55),
                (7.6, 8.6),
                (7.65, 8.65),
                (7.7, 8.7),
                (7.75, 8.75),
                (7.8, 8.8),
                (7.85, 8.85),
                (7.9, 8.9),
                (7.95, 8.95),
                (8, 9),
                (8.05, 9.05),
                (8.1, 9.1),
                (8.15, 9.15),
                (8.2, 9.2),
                (8.25, 9.25),
                (8.3, 9.3),
                (8.35, 9.35),
                (8.4, 9.4),
                (8.45, 9.45),
                (8.5, 9.5),
                (8.55, 9.55),
                (8.6, 9.6),
                (8.65, 9.65),
                (8.7, 9.7),
                (8.75, 9.75),
                (8.8, 9.8),
                (8.85, 9.85),
                (8.9, 9.9),
                (8.95, 9.95),
                (9,9.9),
                (9, 9.85),(9,9.5),
                (9, 9),(9,8.5),(9,8)
            ]
        self.pose = Float64MultiArray()
        self.yaw = ModelStates()
        self.throttle_output=Float64()

        self.rate = rospy.Rate(10)
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.0
        self.dist_ld = 0.5

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.max_velocity = 1.57

        self.robot_theta = 0.0
        self.width = 0.8
        self.time_values = []
        self.error_values = []

        # self.waypoints = []
        # self.x_goal_point = 0.0
        # self.y_goal_point = 0.0
        self.waypoints_plot = None

        # Setup matplotlib for plotting
        # self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.fig, (self.ax1) = plt.subplots(1, figsize=(6, 6))


        # Plot for waypoints
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints')
        self.ax1.set_xlim(-1, 11)  # Set x-axis limits from -10 to 10
        self.ax1.set_ylim(-1, 11)  # Set y-axis limits from -10 to 10
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_plot, = self.ax1.plot([], [], 'b--', label='Waypoints')
        self.robot_position_plot, = self.ax1.plot([], [], 'r^', label='Robot Position', markersize=6)  # Thinner shape
        # Add a plot for the robot's path
        self.past_positions_x = []
        self.past_positions_y = []
        self.robot_path_plot, = self.ax1.plot([], [], 'r-', label='Robot Path')  # Robot path plot
        self.ax1.legend()
    
        # Plot for error vs. time
        # self.ax2.set_xlabel('Time')
        # self.ax2.set_ylabel('Error')
        # self.line, = self.ax2.plot([], [], label='Error vs. Time')
        # self.ax2.legend()

        plt.tight_layout()

    # def tuple_list_callback(self, msg):
    #     self.waypoints = [(msg.a[i], msg.b[i]) for i in range(msg.length)]
    #     self.x_goal_point = msg.a[0]
    #     self.y_goal_point = msg.b[0]
    #     self.waypoints.reverse()
    #     self.update_waypoints_plot()

    def update_pose(self, data:Float64MultiArray):
        self.pose = data
        self.currentx = self.pose.data[7]
        self.currenty = self.pose.data[8]
       


    def update_yaw(self, data:ModelStates):
        self.yaw = data
        orientation = self.yaw.pose[1].orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw

    def pidController(self):
        e= 0.0
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        goal_point = self.waypoints[-1]
        distance_to_goal = np.linalg.norm(np.array((self.currentx, self.currenty)) - np.array(goal_point))
        # print("Goal point:",goal_point)
        if distance_to_goal > 0.01:
            e = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            print("Selected Lookahead Point:", lookahead_point)
            e_past = 0
            if e > 0.1:
                self.integral += e * self.dt
                derivative = (e - e_past) / self.dt
                action = self.kp * e + self.ki * self.integral + self.kd * derivative
                self.throttle_output = self.max_velocity * math.tanh(action)
                e_past = e
                print('Error = ', e)
            else:
                self.throttle_output = 0.0     
        else:
            self.throttle_output = 0.0

        # Append time and error values for plotting
        self.time_values.append(rospy.get_time())
        self.error_values.append(e)

        # Publish to other wheel controllers
        rospy.loginfo('Error = %f', e)

        # Plot error vs. time
        # self.line.set_xdata(self.time_values)
        # self.line.set_ydata(self.error_values)
        # self.ax2.relim()
        # self.ax2.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        return self.throttle_output
    
    def find_lookahead_point(self, robot_position):
        candidate_lookahead_points = []
        max_index = -1
        # print('waypoints:', self.waypoints)
        for i, waypoint in enumerate(self.waypoints):
                distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))

                if distance_to_robot < self.dist_ld and i > max_index:
                        candidate_lookahead_points = [waypoint]
                        max_index = i

        if not candidate_lookahead_points:
                return None  # No valid lookahead point found

        # Calculate distances from candidate lookahead points to the goal
        # distances_to_goal = [np.linalg.norm(np.array(waypoint) - np.array((self.x_goal_point, self.y_goal_point))) for waypoint in candidate_lookahead_points]

        # Find the index of the candidate with the maximum distance to the goal
        max_distance_index = np.argmax(distance_to_robot)

        # Select the lookahead point with the maximum distance to the goal
        lookahead_point = candidate_lookahead_points[max_distance_index]

        return lookahead_point

    def purePursuit(self):
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        # print('Lookahead Point:', lookahead_point)

        # print('waypoints:', self.waypoints)
        if lookahead_point is not None:
            alpha = math.atan2((lookahead_point[1] - self.currenty), (lookahead_point[0] - self.currentx))
            L = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            theta = alpha - self.robot_theta
            dx = L * math.cos(theta)
            Vr = self.pidController() * (1 - self.width * dx / (L * L))
            Vl = self.pidController() * (1 + self.width * dx / (L * L))

            Vr = min(max(Vr, -1.57), 1.57)
            Vl = min(max(Vl, -1.57), 1.57)

            Vr_mapped = int(((Vr + 1.57) / (1.57 * 2)) * 127 + 0.5) 
            Vl_mapped = int(((Vl + 1.57) / (1.57 * 2)) * 127 + 0.5)
            print('Right: ', Vr, ' Mapped Right:', Vr_mapped, ' Left: ', Vl, ' Mapped Left:', Vl_mapped)


            self.velocitylm_publisher.publish(Vl)
            self.velocityrm_publisher.publish(Vr)
            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)
            self.velocitylr_publisher.publish(Vl)
            self.velocityrr_publisher.publish(Vr)
            
            # Plot rover position
            self.plot_rover_position()
            # Give time for plot to update
            plt.pause(0.001)

    def plot_rover_position(self):
        # self.ax1.plot(self.currentx, self.currenty, 'ro')  # Plot current position in red
        # self.robot_position_plot.set_data(self.currentx, self.currenty)  # Update the robot position plot
    # Append the current position to the past positions
        self.past_positions_x.append(self.currentx)
        self.past_positions_y.append(self.currenty)
        
        # Update the robot path plot
        self.robot_path_plot.set_data(self.past_positions_x, self.past_positions_y)
        self.robot_position_plot.set_data(self.currentx, self.currenty)  # Update the current position plot
    
    def update_waypoints_plot(self):
        if self.waypoints:
            self.waypoints_x, self.waypoints_y = zip(*self.waypoints)
        else:
            self.waypoints_x, self.waypoints_y = [], []
        self.waypoints_plot.set_data(self.waypoints_x, self.waypoints_y)
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == '__main__':
    try:
        x = Control()
        while not rospy.is_shutdown():
            # if len(x.waypoints) > 0:
                x.purePursuit()
    except rospy.ROSInterruptException:
        pass
