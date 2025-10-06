import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# Path to the expanded NumPy file, output image, and new NumPy file
expanded_npz_path = 'expanded_occupancy_grid.npy'
output_image_path = 'labeled_sections_with_legend.png'
output_npz_path = 'filtered_occupancy_grid.npy'

# Define a set of colors and their names
COLOR_NAMES = [
    ('Black', (0, 0, 0)),
    ('White', (255, 255, 255)),
]

def load_expanded_grid_from_numpy(npz_path):
    """Load the expanded grid from a NumPy array file."""
    return np.load(npz_path)

def find_and_label_sections(expanded_grid):
    """Find and label contiguous sections of 1 in the grid."""
    # Convert the expanded grid to an 8-bit image for OpenCV
    img = (expanded_grid * 255).astype(np.uint8)

    # Find connected components
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)

    # Create an RGB image to display the labeled sections
    labeled_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    # Map colors to labels
    color_map = {}
    for label in range(1, num_labels):  # Start from 1 to skip the background
        if label == 3:
            color_name, color = ('White', (255, 255, 255))  # Force the third section to be white
        else:
            color_name, color = ('Black', (0, 0, 0))  # All other sections black
        
        # Assign color to the label
        labeled_img[labels == label] = np.array(color)
        color_map[label] = color_name

    # Convert the labeled image to a binary grid
    binary_grid = np.zeros_like(expanded_grid, dtype=np.uint8)
    binary_grid[labeled_img[:, :, 0] == 255] = 1  # Set white regions to 1

    return labeled_img, num_labels - 1, color_map, binary_grid

def plot_labeled_sections(labeled_img, num_sections, color_map):
    """Plot the labeled sections with different colors and show color names."""
    plt.figure(figsize=(10, 20))  # Adjust figure size to match the grid dimensions
    plt.imshow(labeled_img, origin='lower')
    plt.title(f'Labeled Sections (Total Sections: {num_sections})')
    plt.xlabel('X (grid cells)')
    plt.ylabel('Y (grid cells)')
    plt.grid(False)

    # Create a legend for the colors
    legend_elements = [Patch(color=np.array(color)/255, label=name) for name, color in COLOR_NAMES]
    plt.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc='upper left')

    plt.savefig(output_image_path)
    plt.show()

def save_binary_grid(binary_grid, file_path):
    """Save the binary grid as a NumPy array."""
    np.save(file_path, binary_grid)

def main():
    # Load the expanded grid from the NumPy array file
    expanded_grid = load_expanded_grid_from_numpy(expanded_npz_path)
    
    # Find and label sections of 1 in the expanded grid
    labeled_img, num_sections, color_map, binary_grid = find_and_label_sections(expanded_grid)
    
    # Save the binary grid to a new NumPy file
    save_binary_grid(binary_grid, output_npz_path)
    
    # Plot and save the labeled sections image with color names
    plot_labeled_sections(labeled_img, num_sections, color_map)
    
    print(f"Number of sections found: {num_sections}")
    print(f"Binary grid saved to: {output_npz_path}")

if __name__ == '__main__':
    main()
