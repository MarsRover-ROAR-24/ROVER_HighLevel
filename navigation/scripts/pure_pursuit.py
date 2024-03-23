#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt

class Turtle:

        def __init__(self):

                rospy.init_node('turtle_controller', anonymous=True)
                self.velocitylf_publisher = rospy.Publisher('/wheel_lhs_front_velocity_controller/command', Float64, queue_size=10)
                self.velocityrf_publisher = rospy.Publisher('/wheel_rhs_front_velocity_controller/command', Float64, queue_size=10)
                self.velocitylr_publisher = rospy.Publisher('/wheel_lhs_rear_velocity_controller/command', Float64, queue_size=10)
                self.velocityrr_publisher = rospy.Publisher('/wheel_rhs_rear_velocity_controller/command', Float64, queue_size=10)
                self.velocitylm_publisher = rospy.Publisher('/wheel_lhs_mid_velocity_controller/command', Float64, queue_size=10)
                self.velocityrm_publisher = rospy.Publisher('/wheel_rhs_mid_velocity_controller/command', Float64, queue_size=10)
                self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)
    
                self.pose = ModelStates()
                self.throttle_output=Float64()
                
                self.rate = rospy.Rate(10)
                self.kp = 0.7
                self.ki = 0.1
                self.kd = 0.0
                self.dist_ld = 3
                # self.waypoints = [(0, 0), (0.5, 0.5), (0.5,0.75),(1, 1),(1, 1.25),(1, 1.5), (1, 2)] 
                self.waypoints = [(0, 0), (0.5,1),(1.5, 2.5), (2.5, 4), (3.75, 5.5),  (5, 6.5), (6.5, 7), (8, 8), (9, 9)] 

                self.goaly= 9
                self.goalx= 9
                self.dt = 0.1
                self.currentx = 0.0
                self.currenty = 0.0
                self.integral = 0.0
                self.max_velocity = 1.2

                self.robot_theta = 0.0
                self.width = 0.8
                self.time_values = []
                self.error_values = []

                # Setup matplotlib for plotting
                plt.ion()  # Turn on interactive mode
                self.fig, self.ax = plt.subplots()
                self.ax.set_xlabel('Time')
                self.ax.set_ylabel('Error')
                self.line, = self.ax.plot([], [], label='Error vs. Time')
                self.ax.legend()

        def update_pose(self, data:ModelStates):
                self.pose = data
                self.currentx= self.pose.pose[1].position.x 
                self.currenty= self.pose.pose[1].position.y
                orientation = self.pose.pose[1].orientation
                orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
                _, _, yaw = euler_from_quaternion(orientation_list)
                self.robot_theta = yaw


        def pidController(self):
                lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))

                if lookahead_point is not None:
                        e = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
                        print("Selected Lookahead Point:", lookahead_point)
                else:
                        # If no lookahead point found, set the error to the distance between the current position and the goal
                        e = math.hypot(self.goalx - self.currentx, self.goaly - self.currenty)
                        
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
                        
                # Append time and error values for plotting
                self.time_values.append(rospy.get_time())
                self.error_values.append(e)

                # Publish to other wheel controllers
                rospy.loginfo('Error = %f', e)

                # Plot error vs. time
                self.line.set_xdata(self.time_values)
                self.line.set_ydata(self.error_values)
                self.ax.relim()
                self.ax.autoscale_view()
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

                return self.throttle_output


        def find_lookahead_point(self, robot_position):

                candidate_lookahead_points = []

                for waypoint in self.waypoints:
                        distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))
                        
                        if distance_to_robot < self.dist_ld:
                                candidate_lookahead_points.append(waypoint)

                if not candidate_lookahead_points:
                        return None  # No valid lookahead point found

                # Calculate distances from candidate lookahead points to the goal
                distances_to_goal = [np.linalg.norm(np.array(waypoint) - np.array((self.goalx, self.goaly))) for waypoint in candidate_lookahead_points]

                # Find the index of the candidate with the minimum distance to the goal
                min_distance_index = np.argmin(distances_to_goal)

                # Select the lookahead point with the minimum distance to the goal
                lookahead_point = candidate_lookahead_points[min_distance_index]

                return lookahead_point
        
       # def find_adaptive_lookahead_point(self, robot_position, robot_velocity):
                candidate_lookahead_points = []

                for waypoint in self.waypoints:
                        distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))
                        
                        if distance_to_robot < self.max_lookahead_distance:
                                candidate_lookahead_points.append(waypoint)

                if not candidate_lookahead_points:
                        return None  # No valid lookahead point found

                # Calculate distances from candidate lookahead points to the goal
                distances_to_goal = [np.linalg.norm(np.array(waypoint) - np.array((self.goalx, self.goaly))) for waypoint in candidate_lookahead_points]

                # Find the index of the candidate with the minimum distance to the goal
                min_distance_index = np.argmin(distances_to_goal)

                # Select the lookahead point with the minimum distance to the goal
                lookahead_point = candidate_lookahead_points[min_distance_index]

                # Adaptive lookahead distance based on velocity
                adaptive_lookahead_distance = self.min_lookahead_distance + self.velocity_to_lookahead_gain * np.linalg.norm(np.array(robot_velocity))

                # Limit the lookahead distance to a maximum value
                lookahead_distance = min(adaptive_lookahead_distance, self.max_lookahead_distance)

                return lookahead_point, lookahead_distance

     
        def purePursuit(self):
                lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))

                if lookahead_point is not None:
                        alpha = math.atan2((lookahead_point[1] - self.currenty), (lookahead_point[0] - self.currentx))
                        L = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
                        theta = alpha - self.robot_theta
                        dx = L * math.cos(theta)
                        Vr = self.pidController() * (1 - self.width * dx / (L * L))
                        Vl = self.pidController() * (1 + self.width * dx / (L * L))
                        print('Right: ', Vr, ' Left: ', Vl)
                        self.velocitylm_publisher.publish(Vl)
                        self.velocityrm_publisher.publish(-Vr)
                        self.velocitylf_publisher.publish(Vl)
                        self.velocityrf_publisher.publish(-Vr)
                        self.velocitylr_publisher.publish(Vl)
                        self.velocityrr_publisher.publish(Vr)

if __name__ == '__main__':
    try:
        x = Turtle()
        while not rospy.is_shutdown():
        #     x.purePursuit()
        #        time.sleep(2)
               x.purePursuit()
    except rospy.ROSInterruptException:
        pass
    

      