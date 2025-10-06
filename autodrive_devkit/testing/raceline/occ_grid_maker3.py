import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# Path to the expanded NumPy file and output image
expanded_npz_path = 'expanded_occupancy_grid.npy'
output_image_path = 'labeled_sections_with_legend.png'

# Define a set of colors and their names
COLOR_NAMES = [
    ('Red', (255, 0, 0)),
    ('Green', (0, 255, 0)),
    ('Blue', (0, 0, 255)),
    ('Cyan', (0, 255, 255)),
    ('Magenta', (255, 0, 255)),
    ('Yellow', (255, 255, 0)),
    ('Black', (0, 0, 0)),
    ('White', (255, 255, 255)),
    ('Orange', (255, 165, 0)),
    ('Purple', (128, 0, 128)),
    ('Brown', (165, 42, 42)),
    ('Pink', (255, 192, 203)),
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
        if label <= len(COLOR_NAMES):
            color_name, color = COLOR_NAMES[label - 1]
        else:
            color_name, color = COLOR_NAMES[-1]  # Use last color if more labels than colors
        # Normalize color to range [0, 1]
        normalized_color = tuple(c / 255 for c in color)
        color_map[label] = (color_name, normalized_color)
        labeled_img[labels == label] = np.array(color)  # Keep color in [0, 255]

    return labeled_img, num_labels - 1, color_map

def plot_labeled_sections(labeled_img, num_sections, color_map):
    """Plot the labeled sections with different colors and show color names."""
    plt.figure(figsize=(8, 20))  # Adjust figure size to match the grid dimensions
    plt.imshow(labeled_img, origin='lower')
    plt.title(f'Labeled Sections (Total Sections: {num_sections})')
    plt.xlabel('X (400 grid cells for 4 meters)')
    plt.ylabel('Y (1000 grid cells for 10 meters)')
    plt.grid(False)

    # Create a legend for the colors
    legend_elements = [Patch(color=normalized_color, label=f'{name}') for name, normalized_color in color_map.values()]
    plt.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc='upper left')

    plt.savefig(output_image_path)
    plt.show()

def main():
    # Load the expanded grid from the NumPy array file
    expanded_grid = load_expanded_grid_from_numpy(expanded_npz_path)
    
    # Find and label sections of 1 in the expanded grid
    labeled_img, num_sections, color_map = find_and_label_sections(expanded_grid)
    
    # Plot and save the labeled sections image with color names
    plot_labeled_sections(labeled_img, num_sections, color_map)
    
    print(f"Number of sections found: {num_sections}")

if __name__ == '__main__':
    main()
