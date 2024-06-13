import numpy as np
import matplotlib.pyplot as plt

g = [0.3, 0]

points = []
num_points = 1000000
for _ in range(num_points):
    x = np.random.uniform(0, 1)
    y = np.random.uniform(0, 1)
    t = np.random.uniform(np.pi / 4, 3 * np.pi / 4)
    points.append(np.array([x, y, t]))
points = np.array(points)

projected = []
for point in points:
    x = point[0] + g[0] * np.cos(point[2]) - g[1] * np.sin(point[2])
    y = point[1] + g[0] * np.sin(point[2]) + g[1] * np.cos(point[2])
    projected.append(np.array([x, y]))
projected = np.array(projected)

# Set the axis to be from -1 to 1
plt.xlim(-1, 2)
plt.ylim(-1, 2)
# Also set the aspect ratio to be equal
plt.gca().set_aspect('equal', adjustable='box')
plt.scatter(projected[:, 0], projected[:, 1], s=2)
plt.show()