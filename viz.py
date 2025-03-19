import numpy as np
import matplotlib.pyplot as plt

# Load trajectory data
data = np.loadtxt("drone_trajectory.csv", delimiter=",")

# Generate waypoints for reference
t = np.linspace(0, 2 * np.pi, 20)
waypoints = np.vstack((t * 2, np.sin(t) * 5)).T  # (x, y) sine wave trajectory

# Plot the waypoints and drone's actual flight path
plt.figure(figsize=(10, 5))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label="Waypoints")  # Red circles for waypoints
plt.plot(data[:, 0], data[:, 1], 'b-', label="Drone Path")  # Blue line for trajectory

plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Drone Path Following Non-Linear Waypoints")
plt.legend()
plt.grid()
plt.show()
