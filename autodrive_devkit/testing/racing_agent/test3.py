#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.spatial import cKDTree

# Paths to your CSV files
outer_line_csv = '/home/autodrive_devkit/src/racing_agent/inner_line.csv'
clean_outer_line_csv = '/home/autodrive_devkit/src/racing_agent/clean_inner_line.csv'
radius = 0.3  # Radius for downsampling

def load_data(csv_path):
    """Load data from CSV file."""
    x = []
    y = []
    with open(csv_path, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header
        for row in reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
    return np.array(x), np.array(y)

def downsample_points(x, y, radius):
    """Downsample points by merging points within a given radius."""
    points = np.vstack((x, y)).T
    tree = cKDTree(points)
    sampled_points = []

    used_indices = set()

    for i, point in enumerate(points):
        if i in used_indices:
            continue

        # Find all points within the radius of the current point
        indices = tree.query_ball_point(point, radius)
        if indices:
            # Take the average of these points as the representative point
            avg_point = np.mean(points[indices], axis=0)
            sampled_points.append(avg_point)
            used_indices.update(indices)

    return np.array(sampled_points)

def save_csv(csv_path, x, y):
    """Save data to a CSV file."""
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])
        for xi, yi in zip(x, y):
            writer.writerow([xi, yi])

def plot_points(downsampled_x, downsampled_y):
    """Plot the downsampled points and connect them to form a closed loop."""
    plt.figure(figsize=(12, 8))

    # Plot downsampled points
    plt.scatter(downsampled_x, downsampled_y, c='red', s=20, label='Downsampled Points')

    # Plot the line connecting the points and closing the loop
    if len(downsampled_x) > 1:
        plt.plot(np.append(downsampled_x, downsampled_x[0]), 
                 np.append(downsampled_y, downsampled_y[0]), 
                 'g-', lw=2, label='Connected Line')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Downsampled Points with Connected Line')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Load and downsample data
    x, y = load_data(outer_line_csv)
    downsampled_points = downsample_points(x, y, radius)
    downsampled_x, downsampled_y = downsampled_points[:, 0], downsampled_points[:, 1]

    # Save downsampled points to a CSV file
    save_csv(clean_outer_line_csv, downsampled_x, downsampled_y)

    # Plot the results
    plot_points(downsampled_x, downsampled_y)

if __name__ == '__main__':
    main()
