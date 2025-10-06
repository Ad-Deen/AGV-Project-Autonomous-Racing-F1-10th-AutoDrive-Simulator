import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, sqrt
import json

# Path to the .npy file
npy_file_path = 'filtered_occupancy_grid.npy'

# Starting position after applying the transformation
start_x, start_y = 440, 1155

# Orientation and steering constraints
orientation_angle = 4.71  # Starting orientation (radians)
steering_limit = 0.5236  # Steering angle limit in radians
step_size = 110  # Node step size (in grid cells)

# Load the occupancy grid from the .npy file
occupancy_grid = np.load(npy_file_path)

# Global variable to hold transformed centerline points
transformed_centerline_points = []

def load_and_transform_centerline_points(json_file_path):
    """Loads and transforms centerline points."""
    global transformed_centerline_points  # Declare the global variable
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

def euclidean_distance(x1, y1, x2, y2):
    """Calculate the Euclidean distance between two points."""
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

def find_closest_centerline_point(grid_x, grid_y):
    """Finds the closest centerline point and returns its index."""
    distances = np.sqrt((np.array([p[0] for p in transformed_centerline_points]) - grid_x) ** 2 +
                        (np.array([p[1] for p in transformed_centerline_points]) - grid_y) ** 2)
    closest_index = np.argmin(distances)
    return closest_index

def is_valid_node(grid, x, y):
    """Check if the node is within bounds and on a free cell (1 in the grid)."""
    if x < 0 or y < 0 or x >= grid.shape[1] or y >= grid.shape[0]:
        return False
    return grid[int(y), int(x)] == 1

def get_neighbors(x, y, orientation_angle, step_size, steering_limit):
    """Generate valid neighbors within the steering angle constraints."""
    neighbors = []
    for delta_angle in np.linspace(-steering_limit, steering_limit, num=5):  # Emit nodes within steering limit
        new_angle = orientation_angle + delta_angle
        new_x = x + step_size * cos(new_angle)
        new_y = y + step_size * sin(new_angle)
        
        if is_valid_node(occupancy_grid, new_x, new_y):
            neighbors.append((new_x, new_y, new_angle))
    return neighbors

def plot_grid_with_path(occupancy_grid, path_nodes, current_point, iteration, current_goal):
    """Plot the occupancy grid, current node, traversed path."""
    plt.clf()  # Clear the current plot
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')

    # Plot the path as a line
    if len(path_nodes) > 1:
        path_x, path_y = zip(*[(n[0], n[1]) for n in path_nodes])
        plt.plot(path_x, path_y, color='blue', linewidth=2, label='Traversed Path')

    # Plot the current point
    plt.scatter([current_point[0]], [current_point[1]], color='red', label='Current Point', s=50)

    # Plot the current goal
    plt.scatter([current_goal[0]], [current_goal[1]], color='yellow', label='Current Goal', s=100, marker='o')

    plt.title(f'A* Pathfinding - Iteration {iteration}')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    plt.legend()
    plt.grid(False)
    plt.pause(0.1)  # Pause to allow the plot to update

def a_star_pathfinding(start_x, start_y, start_orientation):
    """A* pathfinding loop with cost function."""
    path = []  # List to store the final path
    open_list = [(start_x, start_y, start_orientation, 0)]  # Open list with cost initialized to 0
    closed_list = []  # Closed list to store visited nodes

    plt.ion()  # Enable interactive mode for dynamic plotting
    fig, ax = plt.subplots(figsize=(8, 20))  # Single window for plotting

    while open_list:
        # Sort open list by total cost (g + h)
        open_list.sort(key=lambda x: x[3])  
        current_node = open_list.pop(0)  # Get the node with the least cost
        path.append(current_node)

        # Get the closest centerline point for goal calculation
        closest_index = find_closest_centerline_point(current_node[0], current_node[1])

        # Get neighbors within steering constraints
        neighbors = get_neighbors(current_node[0], current_node[1], current_node[2], step_size, steering_limit)

        for neighbor in neighbors:
            # Calculate the cost from start to this neighbor (g)
            g_cost = euclidean_distance(start_x, start_y, neighbor[0], neighbor[1])  # Actual cost
            # Heuristic cost (h) to goal (using closest centerline point)
            h_cost = euclidean_distance(neighbor[0], neighbor[1],
                                        transformed_centerline_points[closest_index][0],
                                        transformed_centerline_points[closest_index][1])
            total_cost = g_cost + h_cost  # Total cost function (f = g + h)

            if neighbor not in closed_list:
                open_list.append((neighbor[0], neighbor[1], neighbor[2], total_cost))
                closed_list.append(neighbor)

        # Plot the grid, update the path dynamically
        plot_grid_with_path(occupancy_grid, path, current_node, len(path), transformed_centerline_points[closest_index])

        if closest_index >= 0 and g_cost + h_cost < 1:  # Arbitrary condition for completion
            print("Path completed!")
            break

    plt.ioff()  # Disable interactive mode once pathfinding is done
    plt.show()

def main():
    json_file_path = '/home/deen/testing/raceline/updated_centerline.json'
    load_and_transform_centerline_points(json_file_path)  # Load centerline points
    a_star_pathfinding(start_x, start_y, orientation_angle)

if __name__ == '__main__':
    main()
