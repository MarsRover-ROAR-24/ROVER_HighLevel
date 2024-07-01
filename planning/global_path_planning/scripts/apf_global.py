#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import math 

# Constants
K_ATTRACTIVE = 8.0
K_REPULSIVE = 100.0
ROBOT_RADIUS = 1.0  # Minimum distance to obstacles
q_star = 1.4

# Define the attractive potential
def attractive_potential(pos, goal):
    Fx_att = -K_ATTRACTIVE*(pos-goal)
    Fy_att = -K_ATTRACTIVE*(pos-goal)
    return Fx_att,Fy_att

# Define the repulsive potential
def repulsive_potential(pos, obstacles):
    # rep_potential = 0.0
    # for obs in obstacles:
    #     distance = np.linalg.norm(pos - obs)
    #     if distance < ROBOT_RADIUS:
    #         rep_potential += 0.5 * K_REPULSIVE * (1.0 / distance - 1.0 / ROBOT_RADIUS)**2
    d_obs_val = (pos[0] - obstacles[0])**2 + (pos[1] - obstacles[1])**2

    if  d_obs_val< q_star:
        Fx_rep_val = pos[0] + pos[1] - obstacles[0] * obstacles[1] + d_obs_val
        Fy_rep_val = pos[0] - pos[1] + obstacles[0] / obstacles[1] - d_obs_val
    else:
        Fx_rep_val = 0
        Fy_rep_val = 0
    rep_potential = Fx_rep_val + Fy_rep_val    
    return rep_potential

# Tot al potential
def total_potential(pos, goal, obstacles):
    total_potential = float(math.sqrt(attractive_potential(pos, goal)**2 + repulsive_potential(pos, obstacles)**2))
    return total_potential

# Gradient of the total potential
def gradient_potential(pos, goal, obstacles):
    grad = K_ATTRACTIVE * (pos - goal)
    for obs in obstacles:
        distance = np.linalg.norm(pos - obs)
        if distance < ROBOT_RADIUS:
            grad += K_REPULSIVE * (1.0 / distance - 1.0 / ROBOT_RADIUS) * (pos - obs) / (distance**3)
    return grad

# Path planning using APF
def apf_path_planning(start, goal, obstacles, step_size=0.1, max_iters=1000):
    path = [start]
    pos = np.array(start, dtype=np.float64)
    for _ in range(max_iters):
        grad = gradient_potential(pos, goal, obstacles)
        new_pos = pos - step_size * grad
        if np.linalg.norm(new_pos - goal) < step_size:
            path.append(goal)
            break
        path.append(new_pos.tolist())
        pos = new_pos
    print(path)
    return path