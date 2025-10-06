#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import csv

# Paths to your CSV files
outer_line_csv = '/home/autodrive_devkit/src/racing_agent/outer_line.csv'
inner_line_csv = '/home/autodrive_devkit/src/racing_agent/inner_line.csv'

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

def plot_racelines(inner_x, inner_y, outer_x, outer_y):
    """Plot the inner and outer racelines."""
    plt.figure(figsize=(12, 8))

    # Plot the outer raceline
    plt.plot(outer_x, outer_y, 'b-', lw=2, label='Outer Raceline')

    # Plot the inner raceline
    plt.plot(inner_x, inner_y, 'r-', lw=2, label='Inner Raceline')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Inner and Outer Racelines')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Load data from CSV files
    outer_x, outer_y = load_data(outer_line_csv)
    inner_x, inner_y = load_data(inner_line_csv)

    # Plot the results
    plot_racelines(inner_x, inner_y, outer_x, outer_y)

if __name__ == '__main__':
    main()
