#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Int8MultiArray, Float64
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
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

        self.pose = ModelStates()
        self.throttle_output = Float64()
        self.rate = rospy.Rate(10)
        
        self.kp = rospy.get_param("~kp", 0.5)
        self.ki = rospy.get_param("~ki", 0.5)
        self.kd = rospy.get_param("~kd", 0.0)
        self.dist_ld = rospy.get_param("~dist_ld", 0.45)
        
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
        self.waypoints = [
            (0.00, 0.00), (0.00, -0.45), (0.00, -0.90), (0.00, -1.35), (0.00, -1.75), 
            (0.18, -1.75), (0.35, -1.75), (0.50, -1.75), (0.70, -1.85), (0.90, -1.95), 
            (1.00, -2.00), (1.20, -2.00), (1.40, -2.00), (1.50, -2.00), (1.70, -2.00), 
            (1.90, -2.00), (2.00, -2.00), (2.10, -2.10), (2.20, -2.20), (2.30, -2.30), 
            (2.40, -2.40), (2.50, -2.50), (2.60, -2.60), (2.70, -2.70), (2.80, -2.80), (2.90, -2.90), (3, -3.00), 
            (3, -3.20), (3, -3.40), (3.00, -3.50), (3.00, -3.70), (3.00, -3.90), 
            (3.00, -4.00), (3.00, -4.20), (3.00, -4.40), (3.00, -4.50), (3.20, -4.70), 
            (3.40, -4.90), (3.50, -5.00), (3.70, -5.00), (3.90, -5.00), (4.00, -5.00), 
            (4.20, -5.00), (4.40, -5.00), (4.50, -5.00)
        ]
        self.waypoints_plot = None

        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 6))
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints and Robot Path')
        self.ax1.set_xlim(-0.5, 5)
        self.ax1.set_ylim(-5.5, 0.5)
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_plot, = self.ax1.plot([], [], 'b--', label='Waypoints')
        self.robot_position_plot, = self.ax1.plot([], [], 'r^', label='Robot Position', markersize=6)
        self.past_positions_x = []
        self.past_positions_y = []
        self.robot_path_plot, = self.ax1.plot([], [], 'r-', label='Robot Path')
        self.ax1.legend()
        plt.tight_layout()
        self.update_waypoints_plot()

        self.lookahead_point=None

    def update_pose(self, data: ModelStates):
        self.pose = data
        self.currentx = self.pose.pose[1].position.x
        self.currenty = self.pose.pose[1].position.y
        orientation = self.pose.pose[1].orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw

    def map_velocity(self, velocity):
        velocity = min(max(velocity, -1.57), 1.57)
        if velocity < 0:
            return int(((velocity + 1.57) / 1.57) * 61)
        else:
            return int((velocity / 1.57) * 61 + 66)

    def pidController(self):
        e = 0.0
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        if lookahead_point is None:
            return 0.0
        
        goal_point = self.waypoints[-1]
        distance_to_goal = np.linalg.norm(np.array((self.currentx, self.currenty)) - np.array(goal_point))
        e_past = 0

        if distance_to_goal > 0.01:
            e = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            self.integral += e * self.dt
            derivative = (e - e_past) / self.dt
            action = self.kp * e + self.ki * self.integral + self.kd * derivative
            self.throttle_output = self.max_velocity * math.tanh(action)
            e_past = e
        else:
            self.throttle_output = 0.0

        rospy.loginfo('Error = %f', e)
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
            return None

        max_distance_index = np.argmax([np.linalg.norm(np.array(p) - np.array(self.waypoints[-1])) for p in candidate_lookahead_points])
        return candidate_lookahead_points[max_distance_index]

    def purePursuit(self):
        self.lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        print ("Selected point", self.lookahead_point)
        if self.lookahead_point is not None:
            alpha = math.atan2((self.lookahead_point[1] - self.currenty), (self.lookahead_point[0] - self.currentx))
            self.L = math.hypot(self.lookahead_point[0] - self.currentx, self.lookahead_point[1] - self.currenty)
            self.theta = alpha - self.robot_theta

            # Normalize the heading error to the range [0, 360)
            heading_error = (math.degrees(self.theta) + 360) % 360
            rospy.loginfo('Heading error: %f degrees', heading_error)

            # Check if the heading error indicates the next point is behind the rover
            if 300 > heading_error > 240:
                self.rotate_180_in_place()
            else:
                self.move_with_pure_pursuit()

    def rotate_180_in_place(self):
        rospy.loginfo('Starting 180-degree rotation in place...')
        initial_theta = self.robot_theta
        target_theta = (self.robot_theta + math.pi) % (2 * math.pi)

        while True:
            # Calculate current heading error relative to the initial heading
            current_heading_error = (self.robot_theta - initial_theta) % (2 * math.pi)
            
            # Check if the current heading error is within 5 degrees of 180 degrees
            if abs(current_heading_error - math.pi) < math.radians(5):
                rospy.loginfo('Completed 180-degree rotation. Resuming Pure Pursuit')
                break

            # Perform rotation
            Vr = self.max_velocity
            Vl = -self.max_velocity

            Vr_mapped = self.map_velocity(Vr)
            Vl_mapped = self.map_velocity(Vl)
            rospy.loginfo('Rotating in place. Right: %f Mapped Right: %d Left: %f Mapped Left: %d', Vr, Vr_mapped, Vl, Vl_mapped)

            self.velocityrf_publisher.publish(Vr)
            self.velocitylf_publisher.publish(Vl)
            self.velocityrr_publisher.publish(Vr)
            self.velocitylr_publisher.publish(Vl)
            self.velocityrm_publisher.publish(Vr)
            self.velocitylm_publisher.publish(Vl)

            rospy.sleep(0.1)
            rospy.loginfo('Current heading: %f, Initial heading: %f, Heading error: %f', math.degrees(self.robot_theta), math.degrees(initial_theta), math.degrees(current_heading_error))


    def move_with_pure_pursuit(self):
        dx = self.L * math.cos(self.theta)
        Vr = self.pidController() * (1 - self.width * dx / (self.L * self.L))
        Vl = self.pidController() * (1 + self.width * dx / (self.L * self.L))
        Vr = min(max(Vr, -1.57), 1.57)
        Vl = min(max(Vl, -1.57), 1.57)

        Vr_mapped = self.map_velocity(Vr)
        Vl_mapped = self.map_velocity(Vl)
        rospy.loginfo('Moving with Pure Pursuit. Right: %f Mapped Right: %d Left: %f Mapped Left: %d', Vr, Vr_mapped, Vl, Vl_mapped)

        self.velocityrf_publisher.publish(Vr)
        self.velocitylf_publisher.publish(Vl)
        self.velocityrr_publisher.publish(Vr)
        self.velocitylr_publisher.publish(Vl)
        self.velocityrm_publisher.publish(Vr)
        self.velocitylm_publisher.publish(Vl)

        self.past_positions_x.append(self.currentx)
        self.past_positions_y.append(self.currenty)
        self.plot_rover_position()

    def update_waypoints_plot(self):
        self.waypoints_x, self.waypoints_y = zip(*self.waypoints)
        self.waypoints_plot.set_data(self.waypoints_x, self.waypoints_y)
        self.ax1.legend()
        plt.draw()
        plt.pause(0.001)

    def plot_rover_position(self):
        self.robot_position_plot.set_data(self.currentx, self.currenty)
        self.robot_path_plot.set_data(self.past_positions_x, self.past_positions_y)
        plt.draw()
        plt.pause(0.001)

    def main(self):
        while not rospy.is_shutdown():
            self.purePursuit()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        control = Control()
        control.main()
    except rospy.ROSInterruptException:
        pass
