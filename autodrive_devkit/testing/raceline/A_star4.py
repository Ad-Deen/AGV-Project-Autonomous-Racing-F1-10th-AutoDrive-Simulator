import numpy as np
import matplotlib.pyplot as plt
import json

# Global variable to hold transformed centerline points
transformed_centerline_points = []

# Function to load and transform centerline points
def load_and_transform_centerline_points(json_file_path):
    global transformed_centerline_points  # Declare the global variable
    # Load the centerline points from the JSON file
    with open(json_file_path, 'r') as f:
        data = json.load(f)

    # Extract the centerline points
    centerline_points = data["centerline"]

    # Define transformation constants
    transform_x, transform_y = 363, 839

    # Transform the centerline points to grid coordinates
    transformed_centerline_points = [
        (point[0] * 100 + transform_x, point[1] * 100 + transform_y) for point in centerline_points
    ]
    print(transformed_centerline_points)

# Function to find the closest centerline point to a given grid position
def find_closest_centerline_point(grid_x, grid_y):
    distances = np.sqrt((np.array([p[0] for p in transformed_centerline_points]) - grid_x) ** 2 +
                        (np.array([p[1] for p in transformed_centerline_points]) - grid_y) ** 2)
    closest_index = np.argmin(distances)
    print(f"center point index ={closest_index}")
    return transformed_centerline_points[closest_index]

# Load the occupancy grid from the .npy file
npy_file_path = 'filtered_occupancy_grid.npy'
occupancy_grid = np.load(npy_file_path)

# Example usage
json_file_path = '/home/deen/testing/raceline/updated_centerline.json'
input_grid_pos = (440, 1155)

# Load and transform centerline points
load_and_transform_centerline_points(json_file_path)

# Find the closest centerline point
closest_point = find_closest_centerline_point(*input_grid_pos)

# Print the closest centerline point
print(f"Closest centerline point to {input_grid_pos}: {closest_point}")

# Print some details for confirmation
print(f"Occupancy grid shape: {occupancy_grid.shape}")
print(f"Number of centerline points: {len(transformed_centerline_points)}")
print("Sample of transformed centerline points:", transformed_centerline_points[:5])

# Visualize the occupancy grid and transformed centerline points
plt.figure(figsize=(8, 8))  # Adjust figure size as needed

# Plot the occupancy grid
plt.imshow(occupancy_grid, cmap='gray', origin='lower')

# Plot the transformed centerline points on top of the occupancy grid
plt.scatter(*zip(*transformed_centerline_points), color='red', s=10, label='Transformed Centerline Points')

# Highlight the closest point
plt.scatter(*closest_point, color='blue', s=50, label='Closest Point', edgecolor='black')

# Add plot details
plt.title('Occupancy Grid with Transformed Centerline Points')
plt.xlabel('X (grid cells)')
plt.ylabel('Y (grid cells)')
plt.legend()
plt.grid(False)  # Disable grid lines for a cleaner visualization
plt.show()
