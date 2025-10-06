import json
import matplotlib.pyplot as plt
import numpy as np

# Path to the JSON map files
upsampled_map_json_path = '/home/autodrive_devkit/src/racing_agent/upsampled_map.json'
output_png_path = '/home/autodrive_devkit/src/racing_agent/combined_intersections.png'
track_close_json_path = '/home/autodrive_devkit/src/racing_agent/track_close.json'

def load_upsampled_json(json_path):
    """Load upsampled map data from a JSON file."""
    with open(json_path, 'r') as json_file:
        map_data = json.load(json_file)
    return map_data['outer_line'], map_data['inner_line']

def circle_intersections(x1, y1, x2, y2, r):
    """Find the intersection points between two circles."""
    d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)  # Distance between the centers
    
    # Check if circles intersect
    if d > 2 * r or d == 0:
        return None  # No intersection or identical circles
    
    # Find intersection points
    a = (r**2 - r**2 + d**2) / (2 * d)
    h = np.sqrt(r**2 - a**2)
    
    x0 = x1 + a * (x2 - x1) / d
    y0 = y1 + a * (y2 - y1) / d
    
    intersect_x1 = x0 + h * (y2 - y1) / d
    intersect_y1 = y0 - h * (x2 - x1) / d
    
    intersect_x2 = x0 - h * (y2 - y1) / d
    intersect_y2 = y0 + h * (x2 - x1) / d
    
    return (intersect_x1, intersect_y1), (intersect_x2, intersect_y2)

def compute_centerline(inner_intersections, outer_intersections):
    """Compute the centerline from inner and outer intersection points."""
    if len(inner_intersections) != len(outer_intersections):
        raise ValueError("Inner and outer intersections must have the same number of points.")
    
    centerline = [(0.5 * (inner_x + outer_x), 0.5 * (inner_y + outer_y))
                  for (inner_x, inner_y), (outer_x, outer_y) in zip(inner_intersections, outer_intersections)]
    
    return centerline

def plot_combined_intersections(outer_line, inner_line, first_inner_intersections, second_outer_intersections, centerline, output_path):
    """Plot the intersections of inner and outer line circles together without circles."""
    outer_x, outer_y = zip(*outer_line)
    inner_x, inner_y = zip(*inner_line)
    
    plt.figure(figsize=(12, 12))
    
    # Plot the outer raceline with all points connected
    plt.plot(outer_x, outer_y, 'b-', lw=2, label='Upsampled Outer Raceline')
    plt.scatter(outer_x, outer_y, c='b', s=2, edgecolor='k', label='Upsampled Outer Points')
    
    # Plot the inner raceline with all points connected
    plt.plot(inner_x, inner_y, 'r-', lw=2, label='Upsampled Inner Raceline')
    plt.scatter(inner_x, inner_y, c='r', s=2, edgecolor='k', label='Upsampled Inner Points')
    
    radius = 0.55
    
    # Plot intersections for inner line circles
    # for i, (ix1, iy1) in enumerate(first_inner_intersections):
    #     plt.scatter([ix1], [iy1], c='g', s=5, zorder=5, label='1st Inner Intersection Point' if i == 0 else None)
    
    # Plot intersections for outer line circles
    # for i, (ix2, iy2) in enumerate(second_outer_intersections):
    #     plt.scatter([ix2], [iy2], c='m', s=5, zorder=5, label='2nd Outer Intersection Point' if i == 0 else None)
    
    # Plot the new centerline
    center_x, center_y = zip(*centerline)
    plt.plot(center_x, center_y, 'c--', lw=2, label='New Centerline')
    plt.scatter(center_x, center_y, c='c', s=5, edgecolor='k', label='Centerline Points')
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Intersection Points of Inner and Outer Line Circles with New Centerline')
    plt.legend()
    plt.grid(True)
    
    # Set equal scaling for both axes
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save the plot as a PNG file
    plt.savefig(output_path, format='png')
    plt.show()

def save_track_close_json(inner_line, first_inner_intersections, outer_line, second_outer_intersections, centerline, json_path):
    """Save the inner line, inner line intersections, outer line, outer line intersections, and centerline to a JSON file."""
    track_close_data = {
        'inner_line': inner_line,
        'inner_line_intersections': first_inner_intersections,
        'outer_line': outer_line,
        'outer_line_intersections': second_outer_intersections,
        'centerline': centerline
    }
    with open(json_path, 'w') as json_file:
        json.dump(track_close_data, json_file, indent=4)

def main():
    # Load upsampled map data
    outer_line, inner_line = load_upsampled_json(upsampled_map_json_path)
    
    outer_x, outer_y = zip(*outer_line)
    inner_x, inner_y = zip(*inner_line)
    
    # Calculate intersections
    first_inner_intersections = []
    second_outer_intersections = []
    
    radius = 0.55
    
    # Calculate intersections for inner line circles
    for i in range(len(inner_x) - 1):
        x1, y1 = inner_x[i], inner_y[i]
        x2, y2 = inner_x[i + 1], inner_y[i + 1]
        
        intersect = circle_intersections(x1, y1, x2, y2, radius)
        if intersect:
            (ix1, iy1), _ = intersect
            first_inner_intersections.append((ix1, iy1))
    
    # Calculate intersections for outer line circles
    for i in range(len(outer_x) - 1):
        x1, y1 = outer_x[i], outer_y[i]
        x2, y2 = outer_x[i + 1], outer_y[i + 1]
        
        intersect = circle_intersections(x1, y1, x2, y2, radius)
        if intersect:
            _, (ix2, iy2) = intersect
            second_outer_intersections.append((ix2, iy2))
    
    # Compute centerline
    centerline = compute_centerline(first_inner_intersections, second_outer_intersections)
    
    # Save data to JSON file
    save_track_close_json(inner_line, first_inner_intersections, outer_line, second_outer_intersections, centerline, track_close_json_path)
    
    # Plot the intersections and save the plot as a PNG file
    plot_combined_intersections(outer_line, inner_line, first_inner_intersections, second_outer_intersections, centerline, output_png_path)

if __name__ == '__main__':
    main()
