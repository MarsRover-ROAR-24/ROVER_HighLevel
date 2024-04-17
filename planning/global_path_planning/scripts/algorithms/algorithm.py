# from .dijkstra import dijkstra
from .astar import astar
# from .greedy import greedy
# from .q_learning import q_learning
# from .rrt import rrt
# from .planner import planner


def algorithm(start_index, goal_index, width, height, costmap, resolution, origin, viz):
    return astar(start_index, goal_index, width, height, costmap, resolution, origin, viz)
