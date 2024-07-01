#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
import tf2_ros
import geometry_msgs.msg
import tf.transformations

class FramesTransformations:
    '''
    This class is used to put and transform frames in the tf tree
    '''

    def __init__(self):
        '''
        The constructor of the class
        Functionality: This function is used to instantiate the tf2_ros objects
        '''
        # Initialize the ROS node
        rospy.init_node('frames_transformations_node', anonymous=True)

        # This object is used to store the frames
        self.tfBuffer = tf2_ros.Buffer()

        # Instantiate a tf2_ros.TransformListener object
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Instantiate a tf2_ros.StaticTransformBroadcaster object
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Instantiate a tf2_ros.TransformBroadcaster object
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to the /gazebo/model_states topic
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        
        # Store the previous pose to smooth transitions
        self.previous_poses = {}

        # The name of the model you want to track
        self.model_name = "roar"

    def transform(self, parent_id, child_frame_id):
        '''
        Functionality: This function is used to get the transform between two frames and return the pose of the child frame
        Arguments:
            parent_id: name of the parent frame
            child_frame_id: name of the child frame
        Return:
            pose: geometry_msgs.msg.Pose(), list of 6d pose
        '''
        transform_msg = geometry_msgs.msg.TransformStamped()
        pose = geometry_msgs.msg.Pose()
        Pose2List = []

        # Transform the frame
        transform_msg = self.tfBuffer.lookup_transform(parent_id, child_frame_id, rospy.Time(0), timeout=rospy.Duration(0.1))

        # Transfer from TransformStamped() to Pose()
        pose.position.x = transform_msg.transform.translation.x
        pose.position.y = transform_msg.transform.translation.y
        pose.position.z = transform_msg.transform.translation.z
        pose.orientation.x = transform_msg.transform.rotation.x
        pose.orientation.y = transform_msg.transform.rotation.y
        pose.orientation.z = transform_msg.transform.rotation.z
        pose.orientation.w = transform_msg.transform.rotation.w

        # Convert it to 6D list
        angles = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        Pose2List = [pose.position.x, pose.position.y, pose.position.z, angles[0], angles[1], angles[2]]

        return Pose2List

    def move_frame(self, parent_frame_name="map", child_frame_name="base_link", frame_coordinate=[0, 0, 0, 0, 0, 0, 0]):
        '''
        This function is used to put the frame in the tf tree
        Arguments:
            parent_frame_name: name of the parent frame
            child_frame_name: name of the child frame
            frame_coordinate: list of coordinates of the frame 1x6
                [x, y, z, rx, ry, rz, rw]
                translation (in meters) and rotation (in quaternions)
        Functionality:
            This function is used to put the frame in the tf tree
        '''
        frames_msg = geometry_msgs.msg.TransformStamped()
        frames_msg.header.stamp = rospy.Time.now()
        frames_msg.header.frame_id = parent_frame_name
        frames_msg.child_frame_id = child_frame_name
        frames_msg.transform.translation.x = frame_coordinate[0]
        frames_msg.transform.translation.y = frame_coordinate[1]
        frames_msg.transform.translation.z = frame_coordinate[2]
        frames_msg.transform.rotation.x = frame_coordinate[3]
        frames_msg.transform.rotation.y = frame_coordinate[4]
        frames_msg.transform.rotation.z = frame_coordinate[5]
        frames_msg.transform.rotation.w = frame_coordinate[6]

        # Put the frame in the tf tree
        self.broadcaster.sendTransform(frames_msg)

    def model_states_callback(self, data):
        '''
        Callback function for the /gazebo/model_states topic
        '''
        # Find the index of the desired model
        if self.model_name in data.name:
            index = data.name.index(self.model_name)
            pose = data.pose[index]
            position = pose.position
            orientation = pose.orientation
            frame_coordinate = [
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ]

            # Smooth transitions
            if self.model_name in self.previous_poses:
                previous_pose = self.previous_poses[self.model_name]
                frame_coordinate = [
                    (previous_pose[j] + frame_coordinate[j]) / 2.0 for j in range(7)
                ]

            self.previous_poses[self.model_name] = frame_coordinate
            
            # Move the frame
            self.move_frame(parent_frame_name="map", child_frame_name="base_link", frame_coordinate=frame_coordinate)
        else:
            rospy.logwarn("Model %s not found in the current state list.", self.model_name)

if __name__ == '__main__':
    try:
        ft = FramesTransformations()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
