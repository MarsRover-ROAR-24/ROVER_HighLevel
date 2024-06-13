import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev

# Manually inserted waypoints resembling a track with larger curves
waypoints = [
    (0, 0),
    (0.5, 0.25),
    (1, 0.5),
    (1.5, 1),
    (1.8, 1.5),
    (2, 2),
    (2, 4),
    (4, 4),
    (6, 6),
    (7, 6.5),  # Curve start
    (8, 7.5),  # Curve end
    (9, 8.5),
    (9, 9)
]

# Function to add points between waypoints to maintain a maximum distance
def add_points_between_waypoints(waypoints, max_distance=0.05):
    new_waypoints = [waypoints[0]]
    for i in range(1, len(waypoints)):
        x0, y0 = waypoints[i-1]
        x1, y1 = waypoints[i]
        dist = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        if dist > max_distance:
            num_points = int(dist / max_distance) + 1
            for j in range(1, num_points):
                x = x0 + j * (x1 - x0) / num_points
                y = y0 + j * (y1 - y0) / num_points
                new_waypoints.append((x, y))
        new_waypoints.append((x1, y1))
    return new_waypoints

# Add points between waypoints to maintain maximum distance of 0.05
waypoints = add_points_between_waypoints(waypoints)

# Extract x and y coordinates of waypoints
x_coords = [point[0] for point in waypoints]
y_coords = [point[1] for point in waypoints]

# Interpolate the path
tck, _ = splprep([x_coords, y_coords], s=0)
u_new = np.linspace(0, 1, num=1000)
interpolated_points = splev(u_new, tck)

# Plot waypoints and interpolated path
plt.figure(figsize=(10, 8))
plt.plot(interpolated_points[0], interpolated_points[1], 'r', label='Interpolated Path')
plt.plot(x_coords, y_coords, 'bo', label='Waypoints')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Smoothed Path with Maximum Distance of 0.05 between Points')
plt.grid(True)
plt.legend()
plt.show()
