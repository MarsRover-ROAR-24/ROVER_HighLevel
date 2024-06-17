#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Float64,Float64MultiArray, Int8MultiArray
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt

class Control:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/nav_action/supervised', Int8MultiArray, queue_size=10)
        
        self.velocitylf_publisher = rospy.Publisher('/wheel_lhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocityrf_publisher = rospy.Publisher('/wheel_rhs_front_velocity_controller/command', Float64, queue_size=10)
        self.velocitylr_publisher = rospy.Publisher('/wheel_lhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocityrr_publisher = rospy.Publisher('/wheel_rhs_rear_velocity_controller/command', Float64, queue_size=10)
        self.velocitylm_publisher = rospy.Publisher('/wheel_lhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.velocityrm_publisher = rospy.Publisher('/wheel_rhs_mid_velocity_controller/command', Float64, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        # self.path_subscriber = rospy.Subscriber('tuple_list_topic', wp_list, self.tuple_list_callback)  

        self.pose = ModelStates()
        self.throttle_output=Float64()

        self.rate = rospy.Rate(10)
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.0
<<<<<<< HEAD
        self.dist_ld = 0.75
=======
        self.dist_ld = 0.5
>>>>>>> fe986a231483ce935a9deb0f1831ea200a2d15b0

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.max_velocity = 1.57

        self.robot_theta = 0.0
        self.width = 0.8
        self.time_values = []
        self.error_values = []

        self.published_velocity = Int8MultiArray()

<<<<<<< HEAD
        # self.waypoints = []
        self.waypoints =  [(0,0.5),(0,1),(0.5,1.5),(1,2)]
        # (0, 0),
        # (0.5, 0.2), 
        # (1, 0.5),
        # (1.5, 1), 
        # (2, 2), 
        # (2,4),(3.5, 4),(4,4),
        # # (3, 2.5),
        # # (4, 3.5), 
        # (4.5, 4), 
        # (5, 4.5),   # Curve start
        # (6, 5.5),   # Curve end
        # (6,6), 
        # (7, 6.5),   # Curve start
        # (8, 7.5),   # Curve end
        # (9, 8.5),
        # (9.5, 9)
        # ]

=======
        self.waypoints = [
                        (0, 0),
                        (0.125, 0.425),
                        (0.25, 0.85),
                        (0.375, 1.275),
                        (0.5, 1.7),
                        (0.625, 1.85),
                        (0.75, 2),
                        (0.875, 2.15),
                        (1, 2.3),
                        (1.125, 2.35),
                        (1.25, 2.4),
                        (1.375, 2.45),
                        (1.5, 2.5),
                        (1.625, 2.55),
                        (1.75, 2.6),
                        (1.875, 2.65),
                        (2, 2.7),
                        (2.125, 2.7),
                        (2.25, 2.7),
                        (2.375, 2.7),
                        (2.5, 2.7),
                        (2.625, 2.7),
                        (2.75, 2.7),
                        (2.875, 2.7),
                        (3, 2.7),
                        (3.125, 2.65),
                        (3.25, 2.6),
                        (3.375, 2.55),
                        (3.5, 2.5),
                        (3.625, 2.45),
                        (3.75, 2.4),
                        (3.875, 2.35),
                        (4, 2.3),
                        (4.125, 2.25),
                        (4.25, 2.2),
                        (4.375, 2.15),
                        (4.5, 2.1),
                        (4.625, 1.9),
                        (4.75, 1.7),
                        (4.875, 1.275),(4.9,1.175),(4.95,1),(5,0.75),(5,0.6),(5,0.5),(5,0.25),
                        (5, 0)
                        ]

        # self.waypoints =  [(0,1)]
>>>>>>> fe986a231483ce935a9deb0f1831ea200a2d15b0
        # self.x_goal_point = 0.0
        # self.y_goal_point = 0.0
        self.waypoints_plot = None

        # Setup matplotlib for plotting
        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 6))  # Single plot with one axis
        # Plot for waypoints
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints and Robot Path')
        self.ax1.set_xlim(-1, 7)  # Set x-axis limits from -10 to 10
        self.ax1.set_ylim(-2, 5)  # Set y-axis limits from -10 to 10
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_plot, = self.ax1.plot([], [], 'b--', label='Waypoints')
        self.robot_position_plot, = self.ax1.plot([], [], 'r^', label='Robot Position', markersize=6)
        # Add a plot for the robot's path
        self.past_positions_x = []
        self.past_positions_y = []
        self.robot_path_plot, = self.ax1.plot([], [], 'r-', label='Robot Path')  # Robot path plot
        self.ax1.legend()

        plt.tight_layout()
    
        self.update_waypoints_plot()

    # def tuple_list_callback(self, msg):
    #     self.waypoints = [(msg.a[i], msg.b[i]) for i in range(msg.length)]
    #     self.x_goal_point = msg.a[0]
    #     self.y_goal_point = msg.b[0]
    #     self.waypoints.reverse()
    #     self.update_waypoints_plot()

    def update_pose(self, data:ModelStates):
        self.pose = data
        self.currentx = self.pose.pose[1].position.x 
        self.currenty = self.pose.pose[1].position.y
        orientation = self.pose.pose[1].orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw
    
    def map_velocity(self,velocity):
        # Clamping the value to be within the range -1.57 to 1.57
        velocity = min(max(velocity, -1.57), 1.57)
        
        if velocity < 0:
            # Mapping negative values from -1.57 to 0 to the range 0 to 61
            return int(((velocity + 1.57) / 1.57) * 61)
        else:
            # Mapping positive values from 0 to 1.57 to the range 67 to 127
            return int((velocity / 1.57) * 61 + 66)

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
            if e > 0.01:
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
        # self.time_values.append(rospy.get_time())
        # self.error_values.append(e)

        # Publish to other wheel controllers
        rospy.loginfo('Error = %f', e)

        # Plot error vs. time
        # self.line.set_xdata(self.time_values)
        # self.line.set_ydata(self.error_values)
        # self.ax2.relim()
        # self.ax2.autoscale_view()
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()

        return self.throttle_output
    
    def find_lookahead_point(self, robot_position):
        candidate_lookahead_points = []
        max_index = -1

        for i, waypoint in enumerate(self.waypoints):
                distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))

                if distance_to_robot < self.dist_ld and i > max_index:
                        candidate_lookahead_points = [waypoint]
                        max_index = i

        if not candidate_lookahead_points:
                return None  # No valid lookahead point found
        
        # Find the index of the candidate with the maximum distance to the goal
        max_distance_index = np.argmax(distance_to_robot)

        # Select the lookahead point with the maximum distance to the goal
        lookahead_point = candidate_lookahead_points[max_distance_index]

        return lookahead_point

    def purePursuit(self):
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))

        if lookahead_point is not None:
            alpha = math.atan2((lookahead_point[1] - self.currenty), (lookahead_point[0] - self.currentx))
            L = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            theta = alpha - self.robot_theta
            dx = L * math.cos(theta)
            Vr = self.pidController() * (1 - self.width * dx / (L * L))
            Vl = self.pidController() * (1 + self.width * dx / (L * L))

            Vr = min(max(Vr, -1.57), 1.57)
            Vl = min(max(Vl, -1.57), 1.57)

            Vr_mapped = self.map_velocity(Vr)
            Vl_mapped = self.map_velocity(Vl)
            print('Right: ', Vr, ' Mapped Right:', Vr_mapped, ' Left: ', Vl, ' Mapped Left:', Vl_mapped)


            self.velocitylm_publisher.publish(Vl)
            self.velocityrm_publisher.publish(Vr)
            self.velocitylf_publisher.publish(Vl)
            self.velocityrf_publisher.publish(Vr)
            self.velocitylr_publisher.publish(Vl)
            self.velocityrr_publisher.publish(Vr)

            self.published_velocity.data = [Vl_mapped, Vr_mapped, Vl_mapped, Vr_mapped, Vl_mapped, Vr_mapped]
            print ("Mapped Velocities: ", self.published_velocity.data)
            self.velocity_publisher.publish(self.published_velocity)

            # Plot rover position
            self.plot_rover_position()
            # Give time for plot to update
            plt.pause(0.001)

    def plot_rover_position(self):
        # Append the current position to the past positions
        self.past_positions_x.append(self.currentx)
        self.past_positions_y.append(self.currenty)

        # Update the robot path plot
        self.robot_path_plot.set_data(self.past_positions_x, self.past_positions_y)
        self.robot_position_plot.set_data([self.currentx], [self.currenty])  # Update the current position plot

    def update_waypoints_plot(self):
        self.waypoints_x, self.waypoints_y = zip(*self.waypoints)
        self.waypoints_plot.set_data(self.waypoints_x, self.waypoints_y)

if __name__ == '__main__':
    try:
        x = Control()
        while not rospy.is_shutdown():
            if len(x.waypoints) > 0:
                x.purePursuit()
    except rospy.ROSInterruptException:
        pass
