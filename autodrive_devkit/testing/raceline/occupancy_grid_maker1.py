import json
import numpy as np
import matplotlib.pyplot as plt

# Path to the JSON map file
upsampled_map_json_path = 'raceline/updated_centerline.json'
output_png_path = 'occupancy_grid_track_plot.png'
output_npy_path = 'occupancy_grid_track.npy'  # Path to save the occupancy grid as a .npy file

# Define the resolution (1 cm per grid cell)
RESOLUTION = 0.01

def load_upsampled_json(json_path):
    """Load upsampled map data from a JSON file."""
    with open(json_path, 'r') as json_file:
        map_data = json.load(json_file)
    return map_data['outer_line'], map_data['inner_line'], map_data['centerline']

def find_bounding_box(points_list):
    """Find the min and max coordinates in a list of points."""
    min_x = min(point[0] for points in points_list for point in points)
    max_x = max(point[0] for points in points_list for point in points)
    min_y = min(point[1] for points in points_list for point in points)
    max_y = max(point[1] for points in points_list for point in points)
    print(min_x,min_y)
    return min_x, max_x, min_y, max_y

def scale_and_translate_points(points, min_x, min_y, resolution):
    """Translate points without scaling, using the original resolution."""
    scaled_points = []
    
    # Translate each point to grid coordinates using resolution
    for x, y in points:
        grid_x = int((x - min_x) / resolution)
        grid_y = int((y - min_y) / resolution)
        scaled_points.append((grid_x, grid_y))
    
    return scaled_points

def bresenham_line(x0, y0, x1, y1):
    """Generate points on a line using Bresenham's line algorithm."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points

def create_occupancy_grid_with_track(outer_line, inner_line, resolution):
    """Create an occupancy grid where the track lines are plotted as black."""
    # Find the bounding box for scaling
    min_x, max_x, min_y, max_y = find_bounding_box([outer_line, inner_line])
    
    # Dynamically calculate the grid width and height based on bounding box
    grid_width = int((max_x - min_x) / resolution)
    grid_height = int((max_y - min_y) / resolution)

    # Create a blank white occupancy grid (1 for white)
    occupancy_grid = np.ones((grid_height, grid_width), dtype=int)
    
    # Translate the outer and inner lines to fit the grid (no scaling, only translation)
    outer_grid_coords = scale_and_translate_points(outer_line, min_x, min_y, resolution)
    inner_grid_coords = scale_and_translate_points(inner_line, min_x, min_y, resolution)
    
    # Mark the outer and inner lines as black (0 in the grid)
    for i in range(len(outer_grid_coords) - 1):
        x0, y0 = outer_grid_coords[i]
        x1, y1 = outer_grid_coords[i + 1]
        line_points = bresenham_line(x0, y0, x1, y1)
        for x, y in line_points:
            if 0 <= x < grid_width and 0 <= y < grid_height:
                occupancy_grid[y, x] = 0  # Set intermediate points to black
    
    for i in range(len(inner_grid_coords) - 1):
        x0, y0 = inner_grid_coords[i]
        x1, y1 = inner_grid_coords[i + 1]
        line_points = bresenham_line(x0, y0, x1, y1)
        for x, y in line_points:
            if 0 <= x < grid_width and 0 <= y < grid_height:
                occupancy_grid[y, x] = 0  # Set intermediate points to black

    return occupancy_grid

def plot_occupancy_grid(occupancy_grid, output_path):
    """Plot the occupancy grid and save it as a PNG file."""
    plt.figure(figsize=(8, 20))  # Adjust figure size to match the grid dimensions
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')
    plt.title('Occupancy Grid of Racetrack (Track in Black)')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    plt.grid(False)
    plt.savefig(output_path, format='png')
    plt.show()

def save_occupancy_grid_as_npy(occupancy_grid, output_npy_path):
    """Save the occupancy grid as a .npy file."""
    np.save(output_npy_path, occupancy_grid)
    print(f"Occupancy grid saved as {output_npy_path}")

def main():
    # Load upsampled map data
    outer_line, inner_line, centerline = load_upsampled_json(upsampled_map_json_path)
    
    # Create the occupancy grid with the racetrack
    occupancy_grid = create_occupancy_grid_with_track(outer_line, inner_line, RESOLUTION)
    
    # Plot and save the occupancy grid as PNG
    plot_occupancy_grid(occupancy_grid, output_png_path)
    
    # Save the occupancy grid as a .npy file
    save_occupancy_grid_as_npy(occupancy_grid, output_npy_path)

if __name__ == '__main__':
    main()
