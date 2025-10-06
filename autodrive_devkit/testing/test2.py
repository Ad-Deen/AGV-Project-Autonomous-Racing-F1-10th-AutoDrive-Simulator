import json
import matplotlib.pyplot as plt
import numpy as np

# Path to the JSON map file
upsampled_map_json_path = '/home/autodrive_devkit/src/racing_agent/upsampled_map.json'
output_png_path = '/home/autodrive_devkit/src/racing_agent/upsampled_racetrack_map_with_second_intersections.png'

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

def plot_upsampled_map_with_second_intersections(outer_line, inner_line, output_path):
    """Plot the upsampled outer, inner circles, and the second intersection points for the outer line."""
    outer_x, outer_y = zip(*outer_line)
    inner_x, inner_y = zip(*inner_line)
    
    plt.figure(figsize=(12, 12))
    
    # Plot the outer raceline with all points connected
    plt.plot(outer_x, outer_y, 'b-', lw=2, label='Upsampled Outer Raceline')
    plt.scatter(outer_x, outer_y, c='b', s=20, edgecolor='k', label='Upsampled Outer Points')
    
    # Plot the inner raceline with all points connected
    plt.plot(inner_x, inner_y, 'r-', lw=2, label='Upsampled Inner Raceline')
    plt.scatter(inner_x, inner_y, c='r', s=20, edgecolor='k', label='Upsampled Inner Points')
    
    second_intersection_points = []
    
    # Plot circles and calculate only the second intersection points for the outer line
    radius = 0.4
    for i in range(len(outer_x) - 1):
        x1, y1 = outer_x[i], outer_y[i]
        x2, y2 = outer_x[i + 1], outer_y[i + 1]
        
        # Plot circles
        circle1 = plt.Circle((x1, y1), radius, color='m', fill=False, lw=1.5)
        circle2 = plt.Circle((x2, y2), radius, color='m', fill=False, lw=1.5)
        plt.gca().add_patch(circle1)
        plt.gca().add_patch(circle2)
        
        # Calculate intersection points between consecutive circles
        intersect = circle_intersections(x1, y1, x2, y2, radius)
        if intersect:
            _, (ix2, iy2) = intersect  # Only take the second intersection point
            
            second_intersection_points.append((ix2, iy2))
            
            # Plot second intersection point
            plt.scatter([ix2], [iy2], c='m', s=50, zorder=5, label='Second Intersection Point' if i == 0 else None)
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Upsampled Racetrack Map with Second Intersection Points')
    plt.legend()
    plt.grid(True)
    
    # Set equal scaling for both axes
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save the plot as a PNG file
    plt.savefig(output_path, format='png')
    plt.show()

def main():
    # Load upsampled map data
    outer_line, inner_line = load_upsampled_json(upsampled_map_json_path)
    
    # Plot the upsampled map with circles and second intersection points
    plot_upsampled_map_with_second_intersections(outer_line, inner_line, output_png_path)

if __name__ == '__main__':
    main()
