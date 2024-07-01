#!/usr/bin/env python3

# import rospy
# import math
# from sensor_msgs.msg import LaserScan

# def callback(data):
#     # Here you can process the data from LaserScan
#     rospy.loginfo("Received a LaserScan message")
#     rospy.loginfo(f"Range data:{data.ranges}")
#     ranges = data.ranges 

#     if ranges[0] != math.inf:
#         flag = 1
#         rospy.loginfo(f"local_data:{flag}")
#     else:
#         flag = 0
#         rospy.loginfo(f"global_data:{flag}")

            
# def listener():
#     rospy.init_node('laser_scan_listener', anonymous=True)
#     rospy.Subscriber("scan", LaserScan, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         listener()
#     except rospy.ROSInterruptException:
#         pass


############################################################################################################################

import rospy
from map_msgs.msg import OccupancyGridUpdate

class CostmapComparator:
    def __init__(self):
        self.previous_costmap = None
        self.current_costmap = None
        rospy.init_node('costmap_comparator', anonymous=True)
        rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback)
    
    def costmap_callback(self, data):
        self.current_costmap = data.data
        rospy.loginfo(f"{self.current_costmap}")
        
        if self.previous_costmap is not None:
            self.compare_costmaps()
        
        self.previous_costmap = data.data
    
    def compare_costmaps(self):

        for i in self.current_costmap:
            if self.current_costmap[i] == self.previous_costmap[i]:
                self.flag = 0 
                rospy.loginfo("No new obstacles detected")
            else:
                self.flag = 1
                rospy.loginfo("New Obstacles detected")

        
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    comparator = CostmapComparator()
    comparator.run()
