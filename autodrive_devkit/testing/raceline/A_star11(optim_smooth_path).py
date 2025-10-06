import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, radians
import json
import math
from scipy.interpolate import CubicHermiteSpline

# Path to the .npy file
npy_file_path = 'filtered_occupancy_grid.npy'

# Starting position after applying the transformation
start_x, start_y = 440, 1155

# Orientation and steering constraints
orientation_angle = 4.71  # Starting orientation (radians)
steering_limit = 0.5  # Steering angle limit in radians
step_size = 100  # Node step size (in grid cells)

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
        if occupancy_grid[y, x] == 0:  # Assuming occupancy grid uses (y, x) format
            return False
    return True

def plot_smooth_trajectory(points, grid, filename='smooth_trajectory.json'):
    """
    Plots a smooth trajectory based on given waypoints using cubic Hermite splines and saves the trajectory as a JSON file.

    Parameters:
        points (list of tuples): List of waypoints, where each waypoint is a tuple (x, y, orientation).
        grid (2D array): The grid to plot the trajectory over.
        filename (str): The name of the JSON file to save the waypoints to.
    """
    # Extract x, y coordinates and orientation (theta)
    x_vals = np.array([p[0] for p in points])
    y_vals = np.array([p[1] for p in points])
    orientations = np.array([p[2] for p in points])

    # Compute distances between points to assign non-uniform t
    distances = np.sqrt(np.diff(x_vals)**2 + np.diff(y_vals)**2)
    t_vals = np.concatenate([[0], np.cumsum(distances)])

    # Ensure t_vals is strictly increasing by adding a small epsilon if needed
    epsilon = 1e-6
    for i in range(1, len(t_vals)):
        if t_vals[i] <= t_vals[i - 1]:
            t_vals[i] = t_vals[i - 1] + epsilon

    # Convert orientation (theta) to tangents (derivatives)
    dx_vals = np.cos(orientations)  # Tangent x component
    dy_vals = np.sin(orientations)  # Tangent y component

    # Create a cubic Hermite spline for both x and y with tangents
    cubic_spline_x = CubicHermiteSpline(t_vals, x_vals, dx_vals)
    cubic_spline_y = CubicHermiteSpline(t_vals, y_vals, dy_vals)

    # Generate a fine grid of t-values for smooth interpolation (2000 samples)
    t_fine = np.linspace(0, t_vals[-1], 2000)

    # Evaluate the cubic splines at fine t-values
    x_smooth = cubic_spline_x(t_fine)
    y_smooth = cubic_spline_y(t_fine)

    # Combine the smooth trajectory points into a list of (x, y) waypoints
    waypoints = [{'x': int(x), 'y': int(y)} for x, y in zip(x_smooth, y_smooth)]

    # Save waypoints to a JSON file in the local directory
    # with open(filename, 'w') as f:
    #     json.dump(waypoints, f, indent=4)
    
    # print(f"Saved smooth trajectory as {len(waypoints)} waypoints to '{filename}'")

    # Plot the original points and the smoothed path
    plt.figure(figsize=(10, 10))  # Create a square figure
    plt.plot(x_smooth, y_smooth, label='Smoothed Path (Cubic Hermite Spline)', color='blue')
    plt.scatter(x_vals, y_vals, color='red', label='Waypoints')
    plt.imshow(grid, cmap='gray', origin='lower')
    plt.title('Cubic Hermite Spline with Orientation Constraints')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)

    # Set aspect ratio to equal for a square plot
    plt.axis('equal')
    plt.show()


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
    Optimize the path by removing unnecessary intermediate points (zigzags).
    Use the reference point p1 to check if subsequent points form a straight path.
    
    Parameters:
    - p1: [x, y] coordinates of the reference point (starting point).
    - path: List of [x, y] points representing the path.
    - steering_limit: The steering angle limit in radians (default is 0.5236 radians).
    
    Returns:
    - Optimized path with unnecessary points removed.
    """
    i = 0  # Start from the first point
    optim = True  # Flag to keep iterating until no more optimizations are possible

    while optim:
        optim = False  # Assume no optimizations at the beginning of each iteration

        # Iterate through the path, using the current reference point `p1` (initially the start point)
        while i < len(path) - 1:
            p1 = path[i]  # Current reference point

            # Scan forward from the reference point and check possible optimizations
            for j in range(i + 2, len(path)):  # Start from i + 2 to skip direct neighbors
                p2 = path[j]  # Point to check if we can directly connect to p1
                
                # Calculate the angle between p1 and p2
                angle = calculate_angle(p1, p2)
                print(is_line_clear(p1,p2,occupancy_grid))
                # Check if the angle is within the steering limit
                if abs(angle) <= steering_limit and is_line_clear(p1,p2,occupancy_grid) and calculate_angle(p1,path[j+1]) <= steering_limit:
                    # If the angle is within the limit, we can remove intermediate points
                    # Remove points between p1 and p2 (i.e., points at indices i+1 to j-1)
                    for k in range(i + 1, j):
                        print(f"Removing zigzag point: {path[k]} between {p1} and {p2}")
                    path = path[:i + 1] + path[j:]  # Update path by removing intermediate points
                    
                    # Set optimization flag and break to start over with the new path
                    optim = True
                    break

            if optim:
                # If optimizations were made, restart the loop with the updated path
                break

            # Move to the next reference point
            i += 1

    return path  # Return the optimized path

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
    new_path = check_steering_limits(path[0],path,steering_limit)
    print(new_path+[new_path[1]])
    plot_grid_with_path(occupancy_grid, new_path+[new_path[0]], (current_x, current_y), all_nodes, iteration, transformed_centerline_points[closest_index])
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Show the final plot after completion
    plot_smooth_trajectory(new_path+[new_path[1]]+[new_path[2]],occupancy_grid)
    
# Load and transform the centerline points from the JSON file
load_and_transform_centerline_points('/home/deen/testing/raceline/updated_centerline.json')

# Start the A* pathfinding algorithm
a_star_pathfinding(start_x, start_y, orientation_angle)
