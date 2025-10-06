import numpy as np
import matplotlib.pyplot as plt

# Input list of 2D points with orientation
points = [
    (440, 1155, 4.71), (487.85, 283.61, 4.96), (459.14, 172.26, 4.46),
    (380.55, 88.30, 3.96), (271.33, 52.30, 3.46), (158.22, 73.07, 2.96),
    (68.91, 145.52, 2.46), (25.28, 251.92, 1.96), (248.57, 763.13, 1.46),
    (35.81, 1281.90, 1.71), (76.41, 1389.49, 1.21), (163.62, 1464.45, 0.71),
    (276.10, 1488.43, 0.21), (386.29, 1455.54, -0.29), (467.24, 1373.85, -0.79),
    (499.11, 1263.36, -1.29), (487.85, 283.61, 4.96)
]

# Extract x and y coordinates
x_coords = np.array([p[0] for p in points])
y_coords = np.array([p[1] for p in points])

# Function to compute Catmull-Rom spline points
def catmull_rom_spline(P, num_points=100):
    # Number of segments
    n = len(P) - 1
    result = []

    # Iterate through each segment of the curve
    for i in range(n):
        # Get the control points for the segment
        p0 = P[i - 1] if i > 0 else P[i]  # Start point
        p1 = P[i]      # Start of the segment
        p2 = P[i + 1]  # End of the segment
        p3 = P[i + 2] if i < n - 1 else P[i + 1]  # End point

        # Generate points for the current segment
        for t in np.linspace(0, 1, num_points):
            # Catmull-Rom spline equation
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t + 
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t**2 + 
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t**3)
            y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t + 
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t**2 + 
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t**3)
            result.append((x, y))

    return np.array(result)

# Prepare control points for the Catmull-Rom spline
control_points = np.array(list(zip(x_coords, y_coords)))

# Generate Catmull-Rom spline points
spline_points = catmull_rom_spline(control_points, num_points=300)

# Plot the original points and the smoothed Catmull-Rom spline
plt.figure(figsize=(10, 6))
plt.plot(x_coords, y_coords, 'ro-', label='Original Points', markersize=5)
plt.plot(spline_points[:, 0], spline_points[:, 1], 'b-', label='Smoothed Catmull-Rom Spline', linewidth=2)
plt.title('Catmull-Rom Spline Smoothing of 2D Points')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.legend()
plt.grid()
plt.show()
