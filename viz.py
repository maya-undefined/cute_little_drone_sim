import numpy as np
import matplotlib.pyplot as plt

# Load trajectory data
drone_trajectory = np.loadtxt("drone_trajectory.csv", delimiter=",")

t = np.linspace(0, 2 * np.pi, 20)
# evenly space along 2 * pi again
waypoints = np.vstack((t * 2, np.sin(t) * 5, np.sin(t) * 3 + 10)).T  
# (x, y, z) sine wave trajectory
# could probably load these waypoints if they were saved from sim.cpp

fig = plt.figure(figsize=(10, 5))
ax = fig.add_subplot(111, projection='3d')

ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], 'ro-', label="Waypoints") #red
ax.plot(drone_trajectory[:, 0], drone_trajectory[:, 1], drone_trajectory[:, 2], 'b-', label="Drone Path")  #blue

ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_zlabel("Z Position")
ax.set_title("drone_trajectory")
ax.legend()
plt.show()
