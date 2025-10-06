import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, radians
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
    """Finds the closest centerline point and returns its index and progress."""
    distances = np.sqrt((np.array([p[0] for p in transformed_centerline_points]) - grid_x) ** 2 +
                        (np.array([p[1] for p in transformed_centerline_points]) - grid_y) ** 2)
    closest_index = np.argmin(distances)
    return closest_index, calculate_progress(closest_index)

def calculate_progress(closest_index):
    """Calculates progress as a percentage of the total centerline."""
    total_points = len(transformed_centerline_points)
    spawn_index = 93  # Given spawn point index
    if closest_index >= spawn_index:
        progress = (closest_index - spawn_index) / (total_points - spawn_index)
    else:
        progress = (total_points + closest_index - spawn_index) / (total_points - spawn_index)
    return progress/2

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

def plot_grid_with_path(occupancy_grid, path_nodes, current_point, all_nodes, iteration, current_goal):
    """Plot the occupancy grid, current node, traversed path, all emitted nodes."""
    plt.clf()  # Clear the current plot
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')

    # Plot all emitted nodes as small black dots
    if all_nodes:
        nodes_x, nodes_y = zip(*[(n[0], n[1]) for n in all_nodes])
        plt.scatter(nodes_x, nodes_y, color='black', s=0.1, label='Emitted Nodes')  # Small black dots

    # Plot the path as a line
    if len(path_nodes) > 1:
        path_x, path_y = zip(*[(n[0], n[1]) for n in path_nodes])
        plt.plot(path_x, path_y, color='blue', linewidth=2, label='Traversed Path')

    # Plot the current point
    plt.scatter([current_point[0]], [current_point[1]], color='red', label='Current Point', s=50)

    # Plot the current goal
    plt.scatter([current_goal[0]], [current_goal[1]], color='yellow', label='Current Goal', s=100, marker='o')

    plt.title(f'A* Pathfinding with Steering Constraints - Iteration {iteration}')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    plt.legend()
    plt.grid(False)
    plt.pause(0.1)  # Pause to allow the plot to update

def a_star_pathfinding(start_x, start_y, start_orientation):
    """A* pathfinding loop with progress-based heuristic."""
    path = []  # List to store the final path
    all_nodes = []  # List to store all emitted nodes

    plt.ion()  # Enable interactive mode for dynamic plotting
    fig, ax = plt.subplots(figsize=(8, 20))  # Single window for plotting

    open_list = [(start_x, start_y, start_orientation, 0)]  # Open list with heuristic cost initialized to 0
    closed_list = []  # Closed list to store visited nodes

    iteration = 0
    while open_list:
        iteration += 1
        # Sort open_list by heuristic cost (simple priority queue implementation)
        open_list.sort(key=lambda x: x[3])
        current_node = open_list.pop(0)  # Get the current node
        path.append(current_node)

        # Get the closest centerline point for progress calculation
        closest_index, progress = find_closest_centerline_point(current_node[0], current_node[1])
        print(f"Progress: {progress:.2f}%")

        # Check if the progress reached 100% (return to the start point)
        

        # Get neighbors within steering constraints
        neighbors = get_neighbors(current_node[0], current_node[1], current_node[2], step_size, steering_limit)

        # Add neighbors to the open list and closed list to avoid re-visiting
        for neighbor in neighbors:
            if neighbor not in closed_list:
                # Calculate progress and heuristic cost based on (1 - progress)
                _, neighbor_progress = find_closest_centerline_point(neighbor[0], neighbor[1])
                
                heuristic_cost = 1 - neighbor_progress  # New progress-based heuristic
                # if progress > 0.9 :
                    # heuristic_cost = 0.001
                # else:
                    # heuristic_cost = 1 - neighbor_progress  # New progress-based heuristic
                print(f"h ={heuristic_cost}")
                print((neighbor[0], neighbor[1], neighbor[2], heuristic_cost))
                open_list.append((neighbor[0], neighbor[1], neighbor[2], heuristic_cost))
                closed_list.append(neighbor)
                all_nodes.append(neighbor)  # Append to all_nodes for plotting

        # Plot the grid, update the path dynamically, and show emitted nodes
        plot_grid_with_path(occupancy_grid, path, current_node, all_nodes, iteration, transformed_centerline_points[closest_index])
        if progress >= 1.9 and heuristic_cost > 0.5:
            # print(all_nodes)
            print(f"Path completed!")
            break
        if iteration >= 1000:  # Limit iterations for testing
            break

    plt.ioff()  # Disable interactive mode once pathfinding is done
    plt.show()

def main():
    json_file_path = '/home/deen/testing/raceline/updated_centerline.json'
    load_and_transform_centerline_points(json_file_path)  # Load centerline points
    a_star_pathfinding(start_x, start_y, orientation_angle)

if __name__ == '__main__':
    main()
