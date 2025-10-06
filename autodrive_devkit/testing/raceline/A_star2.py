import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, sqrt

# Path to the .npy file
npy_file_path = 'filtered_occupancy_grid.npy'

# Coordinates to spawn the point (after transformation)
start_x, start_y = 440, 1155  # Starting position after applying the transformation (363, 839)

# Goal position
goal_x, goal_y = 405, 84

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

def euclidean_distance(x1, y1, x2, y2):
    """Calculate the Euclidean distance between two points."""
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def plot_grid_with_path(occupancy_grid, path_nodes, current_point, all_nodes):
    """Plot the occupancy grid, current node, traversed path, and all emitted nodes."""
    plt.clf()  # Clear the current plot
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')

    # Plot all emitted nodes as small black dots
    if all_nodes:
        nodes_x, nodes_y = zip(*[(n[0], n[1]) for n in all_nodes])
        plt.scatter(nodes_x, nodes_y, color='black', s=1, label='Emitted Nodes')  # Small black dots

    # Plot the path as a line
    if len(path_nodes) > 1:
        path_x, path_y = zip(*[(n[0], n[1]) for n in path_nodes])
        plt.plot(path_x, path_y, color='blue', linewidth=2, label='Traversed Path')

    # Plot the current point
    plt.scatter([current_point[0]], [current_point[1]], color='red', label='Current Point', s=50)

    # Plot the goal point
    plt.scatter([goal_x], [goal_y], color='green', label='Goal Point', s=100, marker='X')

    plt.title('A* Pathfinding with Steering Constraints')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    plt.legend()
    plt.grid(False)
    plt.pause(0.1)  # Pause to allow the plot to update

def a_star_pathfinding(start_x, start_y, start_orientation):
    """A* pathfinding loop with steering constraints and Euclidean heuristic."""
    open_list = [(start_x, start_y, start_orientation, 0)]  # (x, y, orientation, cost)
    closed_list = set()  # Set to store visited nodes
    path = []  # List to store the final path
    all_nodes = []  # List to store all emitted nodes

    plt.ion()  # Enable interactive mode for dynamic plotting
    fig, ax = plt.subplots(figsize=(8, 20))  # Single window for plotting

    while open_list:
        # Sort open_list by cost (including heuristic)
        open_list.sort(key=lambda node: node[3] + euclidean_distance(node[0], node[1], goal_x, goal_y))
        current_node = open_list.pop(0)  # Get the current node with the lowest cost (cost + heuristic)
        
        # Check if the goal is reached
        if euclidean_distance(current_node[0], current_node[1], goal_x, goal_y) < step_size:
            path.append(current_node)
            break
        
        path.append(current_node)
        closed_list.add((int(current_node[0]), int(current_node[1])))  # Add to closed list

        # Get neighbors within steering constraints
        neighbors = get_neighbors(current_node[0], current_node[1], current_node[2], step_size, steering_limit)
        
        # Add neighbors to the open list if not in closed list
        for neighbor in neighbors:
            neighbor_x, neighbor_y = int(neighbor[0]), int(neighbor[1])
            if (neighbor_x, neighbor_y) not in closed_list:
                # Calculate the cost to reach the neighbor (including heuristic)
                cost = euclidean_distance(start_x, start_y, neighbor[0], neighbor[1])
                open_list.append((neighbor[0], neighbor[1], neighbor[2], cost))
                all_nodes.append(neighbor)  # Append to all_nodes for plotting

        # Plot the grid, update the path dynamically, and show emitted nodes
        plot_grid_with_path(occupancy_grid, path, current_node, all_nodes)

    plt.ioff()  # Disable interactive mode once pathfinding is done
    plt.show()

def main():
    a_star_pathfinding(start_x, start_y, orientation_angle)

if __name__ == '__main__':
    main()
