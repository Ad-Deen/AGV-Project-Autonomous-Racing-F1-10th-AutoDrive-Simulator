import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, radians
import json
import math

# Path to the .npy file
npy_file_path = 'filtered_occupancy_grid.npy'

# Starting position after applying the transformation
start_x, start_y = 440, 1155

# Orientation and steering constraints
orientation_angle = 4.71  # Starting orientation (radians)
steering_limit = 0.5  # Steering angle limit in radians
step_size = 115  # Node step size (in grid cells)

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

def angle_between_points(point1, point2):
    """Returns the angle between two points in radians."""
    return math.atan2(point2[1] - point1[1], point2[0] - point1[0])

def is_line_clear(point1, point2, occupancy_grid):
    """
    Check if the line between point1 and point2 is free of obstacles in the occupancy grid.
    - occupancy_grid: 2D numpy array representing the grid, where 1 = occupied, 0 = free.
    """
    x1, y1 = int(point1[0]), int(point1[1])
    x2, y2 = int(point2[0]), int(point2[1])
    
    # Use Bresenham's line algorithm to trace the line and check for obstacles
    line_points = bresenham_line(x1, y1, x2, y2)
    
    for x, y in line_points:
        if occupancy_grid[y, x] == 1:  # Assuming occupancy grid uses (y, x) format
            return False
    return True

def bresenham_line(x1, y1, x2, y2):
    """Returns the list of points in a line between (x1, y1) and (x2, y2) using Bresenham's algorithm."""
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    
    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    
    return points

def calculate_angle(p1, p2):
    """
    Calculate the angle in radians between two points.
    
    Parameters:
    - p1: [x, y] coordinates of the first point (reference point).
    - p2: [x, y] coordinates of the second point (target point).
    
    Returns:
    - Angle in radians between p1 and p2.
    """
    return  p1[-1] - p2[-1]
    # delta_x = p2[0] - p1[0]
    # delta_y = p2[1] - p1[1]
    # return math.atan2(delta_y, delta_x)

def check_steering_limits(p1, path, steering_limit=steering_limit):
    """
    Check if the angle between point1 and subsequent points is within the steering limit.
    
    Parameters:
    - p1: [x, y] coordinates of the reference point (starting point).
    - path: List of [x, y] points representing the path.
    - steering_limit: The steering angle limit in radians (default is 0.5236 radians).
    
    Returns:
    - List of points that meet the steering condition.
    """
    valid_points = [p1]  # Start with the first point (spawn point)
    index_to_remove = []
    # Assume initial orientation of point1 is aligned with the x-axis (i.e., angle 0)
    p1_orientation = p1[-1]
    optim = True
    while optim:
        index_to_remove = []
        i = i+1
        # Iterate through the path starting from the second point
        for i in range(1, len(path)):
            # Calculate angle between point1 and point[i]
            angle = calculate_angle(p1, path[i])
            
            # Check if the angle is within the steering limit of p1_orientation
            # if (p1_orientation - steering_limit) <= angle <= (p1_orientation + steering_limit):
            if (-steering_limit) <= angle <= (steering_limit):
                valid_points.append(path[i])
                print(f"Valid point found: {path[i]} with angle {angle:.4f} radians")
            else:
                print(f"Point {path[i]} does not meet the steering condition (angle: {angle:.4f} radians).")
        for poi in valid_points[1:-1]:
            index_to_remove.append(valid_points.index(poi))

        if index_to_remove[0] is None:
            optim = False
    return index_to_remove

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
    return progress / 2

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
        plt.plot(path_x, path_y, color='blue', linewidth=2, label='Optimal Path')  # Optimal path in blue

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

def reconstruct_path(parent_map, current_node):
    """Reconstructs the optimal path by backtracking from the current node using parent_map."""
    path = [(current_node[0], current_node[1], current_node[2])]  # Start with the current node
    while parent_map[current_node] is not None:
        current_node = parent_map[current_node]  # Move to the parent
        path.append((current_node[0], current_node[1], current_node[2]))  # Append the parent node with orientation
    path.reverse()  # Reverse the path to get it from start to goal
    return path

def a_star_pathfinding(start_x, start_y, start_orientation):
    """A* pathfinding loop with progress-based heuristic and step-based g cost."""
    path = []  # List to store the final optimal path
    all_nodes = []  # List to store all emitted nodes
    parent_map = {}  # Dictionary to store parent of each node

    plt.ion()  # Enable interactive mode for dynamic plotting
    fig, ax = plt.subplots(figsize=(8, 20))  # Single window for plotting

    # Open list with initial node (x, y, orientation, g_cost, f_cost)
    open_list = [(start_x, start_y, start_orientation, 0, 0)]  # g and f initialized to 0
    closed_list = []  # Closed list to store visited nodes

    # Initialize parent of the start node to None (it has no parent)
    parent_map[(start_x, start_y, start_orientation)] = None

    iteration = 0
    while open_list:
        iteration += 1
        # Sort open_list by f_cost
        open_list.sort(key=lambda x: x[4])  # Sort by f_cost
        current_node = open_list.pop(0)  # Get the current node (x, y, orientation, g_cost, f_cost)
        
        # Extract current node's x, y, orientation
        current_x, current_y, current_orientation, current_g, current_f = current_node

        # Check if this node is a valid part of the path (progress-based check)
        closest_index, progress = find_closest_centerline_point(current_x, current_y)
        print(f"Progress: {progress:.2f}%")

        # Get neighbors within steering constraints
        neighbors = get_neighbors(current_x, current_y, current_orientation, step_size, steering_limit)

        # Add neighbors to the open list and closed list to avoid re-visiting
        for neighbor in neighbors:
            neighbor_x, neighbor_y, neighbor_orientation = neighbor

            if (neighbor_x, neighbor_y, neighbor_orientation) not in closed_list:
                _, neighbor_progress = find_closest_centerline_point(neighbor_x, neighbor_y)

                # Calculate g_cost as the number of steps (assuming step_size is constant)
                g_cost = current_g + step_size  # g = previous g + step size

                # Calculate the f_cost (f = g + h)
                h_cost = (1 - neighbor_progress) * 100  # Progress-based heuristic h
                f_cost = g_cost * 0.01 + h_cost  # Total cost f = g + h

                # Add to open list (neighbor_x, neighbor_y, neighbor_orientation, g_cost, f_cost)
                open_list.append((neighbor_x, neighbor_y, neighbor_orientation, g_cost, f_cost))

                # Add to parent map to allow backtracking
                parent_map[(neighbor_x, neighbor_y, neighbor_orientation)] = (current_x, current_y, current_orientation)

                # Add to all_nodes list for plotting later
                all_nodes.append((neighbor_x, neighbor_y))

        # Add current node to closed list
        closed_list.append((current_x, current_y, current_orientation))
        path = reconstruct_path(parent_map, (current_x, current_y, current_orientation))  # Backtrack to build the path
        # Visualize the grid and pathfinding steps dynamically
        plot_grid_with_path(occupancy_grid, path, (current_x, current_y), all_nodes, iteration, transformed_centerline_points[closest_index])
        
        # If we are at or near the goal (using progress threshold), reconstruct the path
        if progress >= 0.9:
            print(f"Path completed!")
            
            break
            
    plt.show(block=True)
    print(path)
    print(f'experimenting with {path[0]}')
    print(check_steering_limits(path[0],path,steering_limit))

# Load and transform the centerline points from the JSON file
load_and_transform_centerline_points('/home/deen/testing/raceline/updated_centerline.json')

# Start the A* pathfinding algorithm
a_star_pathfinding(start_x, start_y, orientation_angle)
