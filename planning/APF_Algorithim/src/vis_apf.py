#!/usr/bin/env python3

import rospy
import yaml
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import os

def load_map(yaml_file):
    rospy.loginfo(f"Loading map from: {yaml_file}")
    if not os.path.isfile(yaml_file):
        rospy.logerr(f"Map YAML file does not exist: {yaml_file}")
        return None
    
    with open(yaml_file, 'r') as file:
        try:
            map_info = yaml.safe_load(file)
        except yaml.YAMLError as e:
            rospy.logerr(f"Error loading YAML file: {e}")
            return None
    return map_info

def create_potential_field(map_image, map_resolution, map_origin):
    rospy.loginfo("Creating potential field")
    binary_map = np.where(map_image == 0, 1, 0).astype(np.uint8)
    dist_transform = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)
    potential_field = cv2.normalize(dist_transform, None, 0, 1.0, cv2.NORM_MINMAX)
    return potential_field

def downsample_potential_field(potential_field, factor):
    height, width = potential_field.shape
    new_height, new_width = height // factor, width // factor
    downsampled_field = cv2.resize(potential_field, (new_width, new_height), interpolation=cv2.INTER_AREA)
    return downsampled_field

def publish_potential_field(marker_pub, potential_field, map_resolution, map_origin, downsampling_factor):
    rospy.loginfo("Publishing potential field markers")
    marker_array = MarkerArray()
    height, width = potential_field.shape
    
    for y in range(height):
        for x in range(width):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "potential_field"
            marker.id = y * width + x
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = map_origin[0] + x * map_resolution * downsampling_factor
            marker.pose.position.y = map_origin[1] + y * map_resolution * downsampling_factor
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = map_resolution * downsampling_factor
            marker.scale.y = map_resolution * downsampling_factor
            marker.scale.z = potential_field[y, x] * 0.1 + 0.01  # Ensure minimum height

            marker.color.a = 1.0
            marker.color.r = 1.0 - potential_field[y, x]
            marker.color.g = 0.0
            marker.color.b = potential_field[y, x]
            marker_array.markers.append(marker)

    rospy.loginfo(f"Publishing MarkerArray with {len(marker_array.markers)} markers")
    marker_pub.publish(marker_array)

def main():
    rospy.init_node('potential_field_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    
    # Directly specify the path to the YAML file
    yaml_file = "/home/amr/turtlebot_ws/src/APF_Algorithim/APF_Map/map3.yaml"
    rospy.loginfo(f"YAML file path: {yaml_file}")

    map_info = load_map(yaml_file)
    if map_info is None:
        rospy.logerr("Failed to load map information")
        return

    rospy.loginfo(f"Map info: {map_info}")

    map_image_path = os.path.join(os.path.dirname(yaml_file), map_info['image'])
    if not os.path.isfile(map_image_path):
        rospy.logerr(f"Map image file does not exist: {map_image_path}")
        return

    map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
    if map_image is None:
        rospy.logerr(f"Failed to load map image from: {map_image_path}")
        return

    rospy.loginfo(f"Map image shape: {map_image.shape}")

    map_resolution = map_info['resolution']
    map_origin = map_info['origin']
    
    rospy.loginfo(f"Map resolution: {map_resolution}")
    rospy.loginfo(f"Map origin: {map_origin}")
    
    potential_field = create_potential_field(map_image, map_resolution, map_origin)
    
    downsampling_factor = 4
    potential_field = downsample_potential_field(potential_field, downsampling_factor)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        publish_potential_field(marker_pub, potential_field, map_resolution, map_origin, downsampling_factor)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass