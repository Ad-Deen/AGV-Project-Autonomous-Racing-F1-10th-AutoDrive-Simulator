import json
import matplotlib.pyplot as plt
import numpy as np

# Path to the JSON map file
upsampled_map_json_path = 'upsampled_map.json'
output_png_path = 'upsampled_racetrack_map_with_centerline.png'

def load_upsampled_json(json_path):
    """Load upsampled map data from a JSON file."""
    with open(json_path, 'r') as json_file:
        map_data = json.load(json_file)
    return map_data['outer_line'], map_data['inner_line']

def compute_centerline(outer_line, inner_line):
    """Compute the centerline from outer and inner lines."""
    # Ensure both lines have the same number of points
    if len(outer_line) != len(inner_line):
        raise ValueError("Outer and inner lines must have the same number of points.")
    
    centerline = [(0.5 * (outer_x + inner_x), 0.5 * (outer_y + inner_y))
                  for (outer_x, outer_y), (inner_x, inner_y) in zip(outer_line, inner_line)]
    
    return centerline

def plot_upsampled_map(outer_line, inner_line, centerline, output_path):
    """Plot the upsampled outer, inner, and centerline and save the plot as a PNG file."""
    outer_x, outer_y = zip(*outer_line)
    inner_x, inner_y = zip(*inner_line)
    center_x, center_y = zip(*centerline)
    
    plt.figure(figsize=(12, 12))
    
    # Plot the outer raceline with all points connected
    plt.plot(outer_x, outer_y, 'b-', lw=2, label='Upsampled Outer Raceline')
    plt.scatter(outer_x, outer_y, c='b', s=20, edgecolor='k', label='Upsampled Outer Points')
    
    # Plot the inner raceline with all points connected
    plt.plot(inner_x, inner_y, 'r-', lw=2, label='Upsampled Inner Raceline')
    plt.scatter(inner_x, inner_y, c='r', s=20, edgecolor='k', label='Upsampled Inner Points')
    
    # Plot the centerline
    plt.plot(center_x, center_y, 'g--', lw=2, label='Centerline')
    plt.scatter(center_x, center_y, c='g', s=20, edgecolor='k', label='Centerline Points')
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Upsampled Racetrack Map with Centerline')
    plt.legend()
    plt.grid(True)
    
    # Set equal scaling for both axes
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save the plot as a PNG file
    plt.savefig(output_path, format='png')
    plt.show()

def save_upsampled_json(outer_line, inner_line, centerline, json_path):
    """Save the upsampled lines and centerline to a JSON file."""
    upsampled_data = {
        'outer_line': outer_line,
        'inner_line': inner_line,
        'centerline': centerline
    }
    with open(json_path, 'w') as json_file:
        json.dump(upsampled_data, json_file, indent=4)

def main():
    # Load upsampled map data
    outer_line, inner_line = load_upsampled_json(upsampled_map_json_path)
    
    # Compute the centerline
    centerline = compute_centerline(outer_line, inner_line)
    
    # Save the upsampled points and centerline to a new JSON file
    save_upsampled_json(outer_line, inner_line, centerline, upsampled_map_json_path)
    
    # Plot the upsampled map with centerline and save it as a PNG file
    plot_upsampled_map(outer_line, inner_line, centerline, output_png_path)

if __name__ == '__main__':
    main()
