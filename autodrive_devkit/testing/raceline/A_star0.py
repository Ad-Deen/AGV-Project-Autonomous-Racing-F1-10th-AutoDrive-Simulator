import numpy as np
import matplotlib.pyplot as plt

# Path to the .npy file and output image file
npy_file_path = 'filtered_occupancy_grid.npy'
output_png_path = 'occupancy_grid_with_point_and_orientation.png'

# Coordinates to spawn the point (before transformation)
point_x, point_y = 77, 316

# Transformation values
transform_x, transform_y = 363, 839

# Orientation values: [0.00, 0.03, 4.71] rad
# [0.00, 0.03] represents a direction vector, 4.71 is the angle in radians
orientation_angle = 4.71

def load_occupancy_grid(npy_file_path):
    """Load an occupancy grid from a .npy file."""
    return np.load(npy_file_path)

def plot_occupancy_grid_with_point(occupancy_grid, point, orientation_angle, output_path):
    """Plot the occupancy grid with a marked point and orientation arrow."""
    plt.figure(figsize=(8, 20))  # Adjust figure size to match the grid dimensions
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')
    
    # Mark the point on the grid
    plt.scatter([point[0]], [point[1]], color='red', label='Spawn Point', s=100)  # s is the size of the marker

    # Add orientation arrow (using quiver)
    # Calculate direction using cos and sin of the orientation angle
    arrow_length = 20  # You can adjust the arrow length for better visibility
    dx = arrow_length * np.cos(orientation_angle)
    dy = arrow_length * np.sin(orientation_angle)
    plt.quiver(point[0], point[1], dx, dy, angles='xy', scale_units='xy', scale=0.75, color='blue', label='Orientation', width=0.005)

    plt.title('Occupancy Grid with Spawn Point and Orientation Arrow')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    plt.legend()
    plt.grid(False)
    plt.savefig(output_path, format='png')
    plt.show()

def apply_transformation(point, transform_x, transform_y):
    """Apply the transformation to the point."""
    transformed_x = point[0] + transform_x
    transformed_y = point[1] + transform_y
    return transformed_x, transformed_y

def main():
    # Load the occupancy grid from the .npy file
    occupancy_grid = load_occupancy_grid(npy_file_path)
    
    # Apply the transformation to the point
    transformed_point = apply_transformation((point_x, point_y), transform_x, transform_y)
    
    # Plot and save the occupancy grid with the transformed spawn point and orientation arrow
    plot_occupancy_grid_with_point(occupancy_grid, transformed_point, orientation_angle, output_png_path)

if __name__ == '__main__':
    main()
