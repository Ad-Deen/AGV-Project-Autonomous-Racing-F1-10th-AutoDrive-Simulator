import json
import matplotlib.pyplot as plt

# Path to the JSON map file
upsampled_map_json_path = 'raceline/updated_centerline.json'
output_png_path = 'lines_with_centerline_plot.png'
# raceline/updated_centerline.json
def load_upsampled_json(json_path):
    """Load upsampled map data from a JSON file."""
    with open(json_path, 'r') as json_file:
        map_data = json.load(json_file)
    return map_data['outer_line'], map_data['inner_line'], map_data['centerline']

def plot_lines_and_centerline(outer_line, inner_line, centerline, output_path):
    """Plot the outer, inner lines, and centerline."""
    outer_x, outer_y = zip(*outer_line)
    inner_x, inner_y = zip(*inner_line)
    center_x, center_y = zip(*centerline)
    
    plt.figure(figsize=(12, 12))
    
    # Plot the outer raceline with all points connected
    plt.plot(outer_x, outer_y, 'b-', lw=2, label='Outer Raceline')
    
    # Plot the inner raceline with all points connected
    plt.plot(inner_x, inner_y, 'r-', lw=2, label='Inner Raceline')
    
    # Plot the centerline with all points connected
    plt.plot(center_x, center_y, 'g--', lw=2, label='Centerline')
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Outer, Inner Racelines, and Centerline')
    plt.legend()
    plt.grid(True)
    
    # Set equal scaling for both axes
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save the plot as a PNG file
    plt.savefig(output_path, format='png')
    plt.show()

def main():
    # Load upsampled map data
    outer_line, inner_line, centerline = load_upsampled_json(upsampled_map_json_path)
    
    # Plot the lines and centerline and save the plot as a PNG file
    plot_lines_and_centerline(outer_line, inner_line, centerline, output_png_path)

if __name__ == '__main__':
    main()
