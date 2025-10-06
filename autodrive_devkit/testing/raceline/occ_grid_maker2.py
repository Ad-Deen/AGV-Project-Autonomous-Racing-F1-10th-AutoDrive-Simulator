import numpy as np
import cv2
import matplotlib.pyplot as plt

# Path to the saved NumPy file and new file for expanded grid
input_npz_path = 'occupancy_grid_track.npy'
output_npz_path = 'expanded_occupancy_grid.npy'

def load_occupancy_grid_from_numpy(npz_path):
    """Load the occupancy grid from a NumPy array file."""
    return np.load(npz_path)

def expand_zero_regions(occupancy_grid, radius):
    """Expand the 0 regions with a circular radius."""
    # Convert the occupancy grid to an 8-bit image for OpenCV
    img = (occupancy_grid * 255).astype(np.uint8)

    # Create a circular structuring element
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))

    # Create a mask for the 0 regions
    zero_mask = (img == 0).astype(np.uint8)  # Mask where occupancy is 0
    
    # Perform dilation to expand the 0 regions
    expanded_mask = cv2.dilate(zero_mask, kernel, iterations=1)
    
    # Apply the expanded mask to the grid
    expanded_grid = np.where(expanded_mask == 1, 0, img // 255)
    
    return expanded_grid

def plot_expanded_grid(expanded_grid):
    """Plot the expanded grid."""
    plt.figure(figsize=(8, 20))  # Adjust figure size to match the grid dimensions
    plt.imshow(expanded_grid, cmap='gray', origin='lower')
    plt.title('Expanded Occupancy Grid')
    plt.xlabel('X (400 grid cells for 4 meters)')
    plt.ylabel('Y (1000 grid cells for 10 meters)')
    plt.grid(False)
    plt.show()

def save_expanded_grid_to_numpy(expanded_grid, npz_path):
    """Save the expanded grid to a NumPy array file."""
    np.save(npz_path, expanded_grid)

def main():
    # Load the occupancy grid from the NumPy array file
    occupancy_grid = load_occupancy_grid_from_numpy(input_npz_path)
    
    # Expand the zero regions with a circular radius of 20 grids
    expanded_grid = expand_zero_regions(occupancy_grid, radius=5)
    
    # Plot the expanded grid
    plot_expanded_grid(expanded_grid)
    
    # Save the expanded grid to a new NumPy array file
    save_expanded_grid_to_numpy(expanded_grid, output_npz_path)

if __name__ == '__main__':
    main()
