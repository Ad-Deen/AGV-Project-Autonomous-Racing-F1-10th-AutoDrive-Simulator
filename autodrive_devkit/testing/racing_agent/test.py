#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import csv
from sklearn.cluster import DBSCAN
from scipy.interpolate import splprep, splev
import os

# Path to your CSV file
csv_path = '/home/autodrive_devkit/src/racing_agent/map_1.csv'
output_dir = '/home/autodrive_devkit/src/racing_agent/'

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

def cluster_points(x, y, eps=0.5, min_samples=10):
    """Cluster points into two clusters."""
    coords = np.vstack((x, y)).T
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(coords)
    return labels

def fit_curve(x, y):
    """Fit a smooth curve to the points."""
    if len(x) < 2 or len(y) < 2:
        return np.array([]), np.array([])  # Return empty arrays if not enough points

    try:
        tck, u = splprep([x, y], s=0)
        unew = np.linspace(0, 1.0, num=100)  # Fixed number of points for smooth curve
        out = splev(unew, tck)
        return out[0], out[1]
    except Exception as e:
        print(f"Error fitting curve: {e}")
        return np.array([]), np.array([])

def save_to_csv(filename, x, y):
    """Save x and y coordinates to a CSV file."""
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X', 'Y'])
        for xi, yi in zip(x, y):
            writer.writerow([xi, yi])

def plot_racetrack(x, y, labels, raceline1, raceline2):
    """Plot the scattered points, clusters, and fitted curves."""
    plt.figure(figsize=(12, 8))

    # Plot the scattered points
    plt.scatter(x, y, c=labels, cmap='viridis', s=10, label='Scattered Points')
    
    # Plot the fitted curves for inner and outer racelines if available
    if raceline1[0].size > 0 and raceline1[1].size > 0:
        plt.plot(raceline1[0], raceline1[1], 'r-', lw=2, label='Inner Raceline')
    if raceline2[0].size > 0 and raceline2[1].size > 0:
        plt.plot(raceline2[0], raceline2[1], 'b-', lw=2, label='Outer Raceline')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Cleaned Racetrack with Inner and Outer Racelines')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Load and preprocess data
    x, y = load_data(csv_path)
    
    # Cluster the points into two groups
    labels = cluster_points(x, y)

    # Separate the clusters
    cluster1_x = x[labels == 0]
    cluster1_y = y[labels == 0]
    cluster2_x = x[labels == 1]
    cluster2_y = y[labels == 1]

    # Add remaining points to cluster 2 for the inner raceline
    remaining_x = x[labels != 0]
    remaining_y = y[labels != 0]
    cluster2_x = np.concatenate((cluster2_x, remaining_x))
    cluster2_y = np.concatenate((cluster2_y, remaining_y))

    # Fit curves to the clusters
    raceline1 = fit_curve(cluster1_x, cluster1_y)
    raceline2 = fit_curve(cluster2_x, cluster2_y)

    # Save cluster 1 and remaining points to separate CSV files
    outer_line_path = os.path.join(output_dir, 'outer_line.csv')
    remaining_points_path = os.path.join(output_dir, 'remaining_points.csv')

    save_to_csv(outer_line_path, cluster1_x, cluster1_y)
    save_to_csv(remaining_points_path, cluster2_x, cluster2_y)

    # Plot the results
    plot_racetrack(x, y, labels, raceline1, raceline2)

if __name__ == '__main__':
    main()
