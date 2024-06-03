import matplotlib.pyplot as plt
import numpy as np

# Manually inserted waypoints resembling a track with larger curves
waypoints = [(0.0, 1.5),
    (2.5, 3.0),
    (4.5, 6.0),
    (6.5, 4.0),
    (1.0, 8.0),
    (3.5, 2.0),
    (5.0, 7.5),
    (7.5, 9.0),
    (9.0, 5.5),
    (2.0, 6.5),
    (8.5, 1.0),
    (0.5, 4.5),
    (3.0, 5.0),
    (7.0, 3.5),
    (9.5, 0.5),
    (4.0, 9.5),
    (6.0, 8.5),
    (1.5, 7.0),
    (5.5, 2.5),
    (8.0, 6.0)
]

# Extract x and y coordinates of waypoints
x_coords = [point[0] for point in waypoints]
y_coords = [point[1] for point in waypoints]

# Plot waypoints
plt.figure(figsize=(7, 5))
plt.plot(x_coords, y_coords, 'r', label='Waypoints')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Track Waypoints with Larger Curves')
plt.grid(True)
plt.legend()
plt.show()
