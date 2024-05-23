#!/usr/bin/env python3

from PIL import Image

def update_rviz_config(pgm_file):
    img = Image.open(pgm_file)
    width, height = img.size

    # Calculate cell count based on image dimensions or any other desired logic
    cell_count = max(width, height)

    # Update the RViz configuration file
    with open('/home/shams/turtlebot_ws/src/global_path_planning/rviz/turtlebot3_ros.rviz', 'r') as f:
        config_data = f.read()

    config_data = config_data.replace('Plane Cell Count: 512', f'Plane Cell Count: {cell_count}')

    with open('/home/shams/turtlebot_ws/src/global_path_planning/rviz/turtlebot3_ros.rviz', 'w') as f:
        f.write(config_data)

# Example usage
pgm_file_path = '/home/shams/turtlebot_ws/src/ros_world/maps/map2.pgm' #--> change the mapfilename.pgm to change grids.
update_rviz_config(pgm_file_path)

