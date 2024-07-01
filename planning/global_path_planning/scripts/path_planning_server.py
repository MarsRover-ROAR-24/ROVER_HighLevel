#!/usr/bin/env python3

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, Pose
from gridviz import GridViz
from algorithms.algorithm import apf_path_planning
# from pure_pursuit import Turtle
# from final_launch import Control


def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  # costmap as 1-D array representation
  costmap = req.costmap_ros
  # number of columns in the occupancy grid
  width = req.width
  # number of rows in the occupancy grid
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # side of each grid map square in meters
  resolution = 0.05
  # origin of grid map
  origin = [-10.0000, -10.0000, 0.000000]  # here we change the origin according to map.yaml file to make the robot have the same start when picking a point.

  viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)
  # viz = GridViz( resolution, origin)

  # time statistics
  start_time = rospy.Time.now()
  start= [0,0]
  goal = [4,4]
  obstacles = [2,2]
  # calculate the shortes path
  # path = algorithm(start_index, goal_index, width, height, costmap, resolution, origin, viz)
  # path = algorithm(resolution, origin, viz)
  path =apf_path_planning(start, goal, obstacles, step_size=0.1, max_iters=1000)
  if not path:
    rospy.logwarn("No path returned by the path algorithm")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('++++++++ Path Planning execution metrics ++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp


  

def clean_shutdown():
  # cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  # cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  # x=Control()
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    # x.purePursuit()
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)

