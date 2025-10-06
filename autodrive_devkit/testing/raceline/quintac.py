import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline

# Sample points with orientation (x, y, orientation)
points = [
    (440, 1155, 4.71), (487.85, 283.61, 4.96), (459.14, 172.26, 4.46),
    (380.55, 88.30, 3.96), (271.33, 52.30, 3.46), (158.22, 73.07, 2.96),
    (68.91, 145.52, 2.46), (25.28, 251.92, 1.96), (248.57, 763.13, 1.46),
    (35.81, 1281.90, 1.71), (76.41, 1389.49, 1.21), (163.62, 1464.45, 0.71),
    (276.10, 1488.43, 0.21), (386.29, 1455.54, -0.29), (467.24, 1373.85, -0.79),
    (499.11, 1263.36, -1.29), (487.85, 283.61, 4.96), (459.14, 172.26, 4.46),
    (380.55, 88.30, 3.96), (271.33, 52.30, 3.46)
]

# Extract x, y coordinates and orientation (theta)
x_vals = np.array([p[0] for p in points])
y_vals = np.array([p[1] for p in points])
orientations = np.array([p[2] for p in points])

# Compute distances between points to assign non-uniform t
distances = np.sqrt(np.diff(x_vals)**2 + np.diff(y_vals)**2)
t_vals = np.concatenate([[0], np.cumsum(distances)])

# Ensure t_vals is strictly increasing by adding a small epsilon if needed
epsilon = 1e-6
for i in range(1, len(t_vals)):
    if t_vals[i] <= t_vals[i - 1]:
        t_vals[i] = t_vals[i - 1] + epsilon

# Convert orientation (theta) to tangents (derivatives)
dx_vals = np.cos(orientations)  # Tangent x component
dy_vals = np.sin(orientations)  # Tangent y component

# Create a cubic Hermite spline for both x and y with tangents
cubic_spline_x = CubicHermiteSpline(t_vals, x_vals, dx_vals)
cubic_spline_y = CubicHermiteSpline(t_vals, y_vals, dy_vals)

# Generate a fine grid of t-values for smooth interpolation
t_fine = np.linspace(0, t_vals[-1], 1000)

# Evaluate the cubic splines at fine t-values
x_smooth = cubic_spline_x(t_fine)
y_smooth = cubic_spline_y(t_fine)

# Plot the original points and the smoothed path
plt.figure(figsize=(6,15))
plt.plot(x_smooth, y_smooth, label='Smoothed Path (Cubic Hermite Spline)', color='blue')
plt.scatter(x_vals, y_vals, color='red', label='Waypoints')

plt.title('Cubic Hermite Spline with Orientation Constraints')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
