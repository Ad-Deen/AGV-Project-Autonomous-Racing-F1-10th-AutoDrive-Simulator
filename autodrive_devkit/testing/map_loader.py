import json
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

# Path to the JSON map file
map_json_path = '/home/autodrive_devkit/src/racing_agent/racetrack_map.json'
output_png_path = '/home/autodrive_devkit/src/racing_agent/racetrack_map.png'
upsampled_map_json_path = '/home/autodrive_devkit/src/racing_agent/upsampled_map.json'

def load_map_json(json_path):
    """Load map data from a JSON file."""
    with open(json_path, 'r') as json_file:
        map_data = json.load(json_file)
    return map_data['outer_line'], map_data['inner_line']

def close_track(line):
    """Ensure the track boundary is closed by connecting the last point to the first point."""
    if line[0] != line[-1]:
        line.append(line[0])  # Append the first point to the end
    return line

def upsample_line(line, num_points):
    """Upsample a line to a specific number of equally spaced points."""
    x, y = zip(*line)
    dist = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cumulative_dist = np.concatenate(([0], np.cumsum(dist)))
    total_dist = cumulative_dist[-1]
    target_dist = np.linspace(0, total_dist, num_points)
    interp_x = interp1d(cumulative_dist, x, kind='linear')
    interp_y = interp1d(cumulative_dist, y, kind='linear')
    new_x = interp_x(target_dist)
    new_y = interp_y(target_dist)
    return list(zip(new_x, new_y))

def plot_map(outer_line, inner_line, upsampled_outer_line, upsampled_inner_line, output_path):
    """Plot the outer and inner racelines and save the plot as a PNG file."""
    outer_x, outer_y = zip(*outer_line)
    inner_x, inner_y = zip(*inner_line)
    upsampled_outer_x, upsampled_outer_y = zip(*upsampled_outer_line)
    upsampled_inner_x, upsampled_inner_y = zip(*upsampled_inner_line)
    
    plt.figure(figsize=(12, 12))
    
    # Plot the outer raceline with closed loop
    plt.plot(outer_x, outer_y, 'b-', lw=2, label='Outer Raceline')
    plt.scatter(outer_x, outer_y, c='b', s=20, edgecolor='k', label='Outer Points')
    
    # Plot the inner raceline with closed loop
    plt.plot(inner_x, inner_y, 'r-', lw=2, label='Inner Raceline')
    plt.scatter(inner_x, inner_y, c='r', s=20, edgecolor='k', label='Inner Points')
    
    # Plot the upsampled outer raceline
    plt.plot(upsampled_outer_x, upsampled_outer_y, 'b--', lw=2, label='Upsampled Outer Raceline')
    plt.scatter(upsampled_outer_x, upsampled_outer_y, c='b', s=50, edgecolor='k', marker='x', label='Upsampled Outer Points')
    
    # Plot the upsampled inner raceline
    plt.plot(upsampled_inner_x, upsampled_inner_y, 'r--', lw=2, label='Upsampled Inner Raceline')
    plt.scatter(upsampled_inner_x, upsampled_inner_y, c='r', s=50, edgecolor='k', marker='x', label='Upsampled Inner Points')
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Racetrack Map with Upsampled Points')
    plt.legend()
    plt.grid(True)
    
    # Set equal scaling for both axes
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save the plot as a PNG file
    plt.savefig(output_path, format='png')
    plt.show()

def save_upsampled_json(outer_line, inner_line, upsampled_outer_line, upsampled_inner_line, json_path):
    """Save the upsampled lines to a JSON file."""
    upsampled_data = {
        'outer_line': upsampled_outer_line,
        'inner_line': upsampled_inner_line
    }
    with open(json_path, 'w') as json_file:
        json.dump(upsampled_data, json_file, indent=4)

def main():
    # Load map data
    outer_line, inner_line = load_map_json(map_json_path)
    
    # Close the track boundaries
    outer_line = close_track(outer_line)
    inner_line = close_track(inner_line)
    
    # Upsample the lines
    upsampled_outer_line = upsample_line(outer_line, 200)
    upsampled_inner_line = upsample_line(inner_line, 200)
    
    # Save the upsampled points to a new JSON file
    save_upsampled_json(outer_line, inner_line, upsampled_outer_line, upsampled_inner_line, upsampled_map_json_path)
    
    # Plot the map with upsampled points and save it as a PNG file
    plot_map(outer_line, inner_line, upsampled_outer_line, upsampled_inner_line, output_png_path)

if __name__ == '__main__':
    main()
