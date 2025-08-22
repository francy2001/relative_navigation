import matplotlib.pyplot as plt
import numpy as np

def load_points(filename):
    points = []
    with open(filename) as f:
        for line in f:
            if "x=" in line:
                parts = line.strip().split(",")
                x = float(parts[0].split("=")[-1])
                y = float(parts[1].split("=")[-1])
                z = float(parts[2].split("=")[-1])
                points.append((x, y, z))
    return np.array(points)

file = load_points("point_cloud.txt")

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(projection='3d')
#ax.plot(file[:,0], file[:,1], file[:,2], zdir='z', label='Point cloud')

ax.scatter(file[:,0], file[:,1], file[:,2], s=2, c='g', label="point cloud")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("Point Cloud Visualization")
ax.legend()
#plt.figure(figsize=(8,8))
#plt.scatter(file[:,0], file[:,1], s=2, c='g', label="point cloud")
#plt.xlabel("X [m]")
#plt.ylabel("Y [m]")
#plt.zlabel("Z [m]")
#plt.axis("equal")
#plt.legend()
plt.show()
