#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import csv

# Path to your CSV file
csv_path = '/home/autodrive_devkit/src/racing_agent/map_1.csv'

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

def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def iterative_clustering(x, y, threshold=1.0):
    """Cluster points iteratively based on proximity."""
    data = np.vstack((x, y)).T
    clusters = []
    assigned = np.zeros(len(data), dtype=bool)

    for i, point in enumerate(data):
        if not assigned[i]:
            # Start a new cluster
            cluster = [point]
            assigned[i] = True
            
            # Iteratively add points to the cluster
            to_check = [point]
            while to_check:
                current_point = to_check.pop()
                for j, candidate in enumerate(data):
                    if not assigned[j] and distance(current_point, candidate) < threshold:
                        cluster.append(candidate)
                        assigned[j] = True
                        to_check.append(candidate)
            
            clusters.append(np.array(cluster))
    
    return clusters

def plot_clusters(clusters):
    """Plot the clusters."""
    plt.figure(figsize=(10, 10))
    colors = plt.cm.jet(np.linspace(0, 1, len(clusters)))
    for idx, cluster in enumerate(clusters):
        plt.scatter(cluster[:, 0], cluster[:, 1], s=0.5, color=colors[idx], label=f'Cluster {idx}')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Track Visualization with Custom Clusters')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    x, y = load_data(csv_path)
    clusters = iterative_clustering(x, y, threshold=1.0)
    plot_clusters(clusters)

if __name__ == '__main__':
    main()
