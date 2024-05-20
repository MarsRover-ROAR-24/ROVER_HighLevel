#!/usr/bin/env python3

import rospy
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Bool

class CostmapComparator:
    def __init__(self):
        self.previous_costmap = None
        self.current_costmap = None
        rospy.init_node('costmap_comparator', anonymous=True)
        rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback)
        self.flag_pub = rospy.Publisher('/obstacle_flag', Bool, queue_size=10)
    
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
        
        self.flag_pub.publish(self.flag)        

        
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    comparator = CostmapComparator()
    comparator.run()
