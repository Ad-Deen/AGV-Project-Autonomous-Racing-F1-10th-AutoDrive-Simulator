import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians, sqrt

# Path to the .npy file
npy_file_path = 'filtered_occupancy_grid.npy'

# Coordinates to spawn the point (after transformation)
start_x, start_y = 440, 1155  # Starting position after applying the transformation (363, 839)

# Orientation and steering constraints
orientation_angle = 4.71  # Starting orientation (radians)
steering_limit = 0.523  # Steering angle limit in radians
step_size = 40  # Node step size (in grid cells)

# Load the occupancy grid from the .npy file
occupancy_grid = np.load(npy_file_path)

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

def plot_grid_with_path(occupancy_grid, path_nodes, current_point, all_nodes):
    """Plot the occupancy grid, current node, traversed path, and all emitted nodes."""
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

    plt.title('A* Pathfinding with Steering Constraints')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    plt.legend()
    plt.grid(False)
    plt.pause(0.1)  # Pause to allow the plot to update

def a_star_pathfinding(start_x, start_y, start_orientation):
    """A* pathfinding loop with steering constraints."""
    open_list = [(start_x, start_y, start_orientation)]  # Open list initialized with the start node
    closed_list = []  # Closed list to store visited nodes
    path = []  # List to store the final path
    all_nodes = []  # List to store all emitted nodes

    plt.ion()  # Enable interactive mode for dynamic plotting
    fig, ax = plt.subplots(figsize=(8, 20))  # Single window for plotting

    iteration = 0
    while open_list:
        iteration += 1
        current_node = open_list.pop(0)  # Get the current node (for now we use simple queue logic)
        path.append(current_node)
        
        # Get neighbors within steering constraints
        neighbors = get_neighbors(current_node[0], current_node[1], current_node[2], step_size, steering_limit)
        
        # Add neighbors to the open list and closed list to avoid re-visiting
        for neighbor in neighbors:
            if neighbor not in closed_list:
                open_list.append(neighbor)
                closed_list.append(neighbor)
                all_nodes.append(neighbor)  # Append to all_nodes for plotting

        # Plot the grid, update the path dynamically, and show emitted nodes
        plot_grid_with_path(occupancy_grid, path, current_node, all_nodes)
        
        if iteration >= 10:  # Limit iterations for testing
            break
    
    plt.ioff()  # Disable interactive mode once pathfinding is done
    plt.show()

def main():
    a_star_pathfinding(start_x, start_y, orientation_angle)

if __name__ == '__main__':
    main()
