import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# Define a function to smooth the path using B-spline interpolation
def bspline_smoothing_2d(points, num_spline_points=2000, degree=3):
    x_vals, y_vals = points[:, 0], points[:, 1]
    
    # Create parameter t for the curve (cumulative distance between points)
    t = np.linspace(0, 1, len(points))
    
    # Define new t-values for smoother interpolation
    t_smooth = np.linspace(0, 1, num_spline_points)
    
    # Generate B-spline for both x and y values
    spline_x = make_interp_spline(t, x_vals, k=degree)
    spline_y = make_interp_spline(t, y_vals, k=degree)
    
    # Get smoothed x and y coordinates
    x_smooth = spline_x(t_smooth)
    y_smooth = spline_y(t_smooth)
    
    return x_smooth, y_smooth

# Input list of tuples: (x_coordinate, y_coordinate, orientation)
input_points = [
    (440, 1155, 4.71), (487.85541734537486, 283.6143254544342, 4.96),
    (459.13785150518584, 172.25768482526288, 4.46), (380.54859909310215, 88.30097332874344, 3.96),
    (271.3290499956001, 52.299722031901865, 3.46), (158.21995899668934, 73.06829283973865, 2.96),
    (68.91437639036064, 145.52181528926627, 2.46), (25.277423439722874, 251.92114017990562, 1.96),
    (248.57286799333642, 763.1319548284282, 1.46), (35.81437200479505, 1281.8977899117358, 1.71),
    (76.41160314501803, 1389.493630090375, 1.21), (163.62321888392648, 1464.4545137578518, 0.71),
    (276.0967740772035, 1488.4274022401532, 0.20999999999999996), (386.2948197611637, 1455.5428963530971, -0.29000000000000004),
    (467.23703106117085, 1373.8522700250724, -0.79), (499.105931692675, 1263.356237641374, -1.29),
    (487.85541734537486, 283.6143254544342, 4.96)
]

# Convert the list of tuples to a NumPy array for easier manipulation
points_array = np.array(input_points)

# Smooth the 2D points using B-spline
x_smooth, y_smooth = bspline_smoothing_2d(points_array[:, :2])

# Plotting the results
plt.figure(figsize=(10, 6))

# Plot original 2D points and connecting line
plt.plot(points_array[:, 0], points_array[:, 1], 'o-', label="Original Path", color='blue')

# Plot smoothed B-spline curve
plt.plot(x_smooth, y_smooth, label="B-Spline Smoothed Path", color='red')

# Mark the original points
plt.scatter(points_array[:, 0], points_array[:, 1], color='blue')

# Labels and legend
plt.title("2D Points Path Smoothing with B-Spline")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.legend()
plt.grid(True)
plt.axis('equal')  # Keep the aspect ratio equal
plt.show()
