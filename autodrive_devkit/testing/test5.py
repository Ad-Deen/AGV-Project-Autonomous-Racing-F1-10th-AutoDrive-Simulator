#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.interpolate import splprep, splev

# Paths to your CSV files
outer_path = '/home/autodrive_devkit/src/racing_agent/clean_outer_line.csv'
inner_path = '/home/autodrive_devkit/src/racing_agent/clean_inner_line.csv'

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

def fit_curve(x, y):
    """Fit a smooth curve to the points and ensure the curve is closed."""
    if len(x) < 2 or len(y) < 2:
        return np.array([]), np.array([])  # Return empty arrays if not enough points

    try:
        # Ensure the curve is closed
        if not np.array_equal(x[0], x[-1]) or not np.array_equal(y[0], y[-1]):
            x = np.append(x, x[0])
            y = np.append(y, y[0])
        
        tck, u = splprep([x, y], s=0, per=True)  # `per=True` ensures the curve is closed
        unew = np.linspace(0, 1.0, num=100)  # Fixed number of points for smooth curve
        out = splev(unew, tck)
        return out[0], out[1]
    except Exception as e:
        print(f"Error fitting curve: {e}")
        return np.array([]), np.array([])

def plot_racetracks(inner_x, inner_y, outer_x, outer_y, inner_curve_x, inner_curve_y, outer_curve_x, outer_curve_y):
    """Plot the smooth curves for inner and outer boundaries."""
    plt.figure(figsize=(12, 8))

    # Plot the smooth inner and outer racelines
    if outer_curve_x.size > 0 and outer_curve_y.size > 0:
        plt.plot(outer_curve_x, outer_curve_y, 'b-', lw=2, label='Smooth Outer Boundary')
    if inner_curve_x.size > 0 and inner_curve_y.size > 0:
        plt.plot(inner_curve_x, inner_curve_y, 'r-', lw=2, label='Smooth Inner Boundary')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Smoothed Racetrack Map with Inner and Outer Boundaries')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Load data from CSV files
    outer_x, outer_y = load_data(outer_path)
    inner_x, inner_y = load_data(inner_path)
    
    # Fit smooth curves to the outer and inner boundaries
    outer_curve_x, outer_curve_y = fit_curve(outer_x, outer_y)
    inner_curve_x, inner_curve_y = fit_curve(inner_x, inner_y)

    # Plot the results
    plot_racetracks(inner_x, inner_y, outer_x, outer_y, inner_curve_x, inner_curve_y, outer_curve_x, outer_curve_y)

if __name__ == '__main__':
    main()
