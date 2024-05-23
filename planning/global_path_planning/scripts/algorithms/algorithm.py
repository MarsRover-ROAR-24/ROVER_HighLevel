from .astar import astar
# from .APF import waypoints



def algorithm(start_index, goal_index, width, height, costmap, resolution, origin, viz):
    return astar(start_index, goal_index, width, height, costmap, resolution, origin, viz)

# def algorithm(start_index, goal_index, width, height, costmap, resolution, origin, viz):
#     return waypoints