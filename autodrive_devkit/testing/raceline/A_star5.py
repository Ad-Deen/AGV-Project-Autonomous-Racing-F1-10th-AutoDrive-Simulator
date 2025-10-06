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
    # print(transformed_centerline_points)

# Function to find the closest centerline point to a given grid position and calculate progress
def find_closest_centerline_point(grid_x = 100, grid_y = 310):

    # Spawn point index (93rd centerline point)
    spawn_point_index = 93
    distances = np.sqrt((np.array([p[0] for p in transformed_centerline_points]) - grid_x) ** 2 +
                        (np.array([p[1] for p in transformed_centerline_points]) - grid_y) ** 2)
    
    # Find the closest centerline point index
    closest_index = np.argmin(distances)
    print(f"Center point index = {closest_index}")
    
    # Calculate progress relative to spawn point (index 93)
    total_points = len(transformed_centerline_points)
    
    if closest_index >= spawn_point_index:
        progress = (closest_index - spawn_point_index) / (total_points - spawn_point_index)
    else:
        progress = (closest_index + (total_points - spawn_point_index)) / (total_points - spawn_point_index)

    # Normalize progress to a percentage (0-100%)
    progress_percentage = progress/2 *100
    print(f"Progress along the track: {progress_percentage:.2f}%")

    return transformed_centerline_points[closest_index]

# Load the occupancy grid from the .npy file
npy_file_path = 'filtered_occupancy_grid.npy'
occupancy_grid = np.load(npy_file_path)

# Example usage
json_file_path = '/home/deen/testing/raceline/updated_centerline.json'
input_grid_pos = (100, 310)

# Load and transform centerline points
load_and_transform_centerline_points(json_file_path)

# Find the closest centerline point and calculate progress
closest_point = find_closest_centerline_point(*input_grid_pos)

# Print the closest centerline point
print(f"Closest centerline point to {input_grid_pos}: {closest_point}")

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
