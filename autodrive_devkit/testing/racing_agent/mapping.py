#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arrow
import math
import csv
import signal
import sys

class LidarTrackMapper(Node):
    def __init__(self):
        super().__init__('lidar_track_mapper')

        # Subscribe to LiDAR and pose topics
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.scan_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            Float32MultiArray,
            '/bot_pose',
            self.pose_callback,
            10
        )

        # Initialize matplotlib figure for live plotting
        self.fig, self.ax = plt.subplots()
        self.scat = self.ax.scatter([], [], s=0.1, label='LiDAR Points')
        self.position_plot, = self.ax.plot([], [], 'ro', label='Robot Position')
        self.arrow = None
        self.timer = self.create_timer(0.1, self.update_plot)

        # Initialize attributes
        self.filtered_scan_data = None
        self.filtered_scan_angles = None
        self.position = None
        self.orientation = None
        self.lidar_points_x = []
        self.lidar_points_y = []

        # Track robot's initial pose
        self.initial_position = None
        self.initial_pose_set = False

        # Initialize map save path
        self.csv_path = "/home/autodrive_devkit/src/racing_agent/map_1.csv"
        self.png_path = "/home/autodrive_devkit/src/racing_agent/map_1.png"

        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.handle_shutdown)

    def handle_shutdown(self, signum, frame):
        """Handle Ctrl+C signal (SIGINT) and save map data before shutdown."""
        self.get_logger().info('Shutdown signal received. Saving map...')
        self.save_map()
        sys.exit(0)

    def save_map(self):
        """Save the map to CSV and PNG."""
        # Save the map to a CSV file
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y'])  # Column headers
            for xi, yi in zip(self.lidar_points_x, self.lidar_points_y):
                writer.writerow([xi, yi])

        # Save the plot as a PNG file
        self.fig.savefig(self.png_path)
        self.get_logger().info(f"Map saved to {self.csv_path} and {self.png_path}")

    def scan_callback(self, msg):
        """Callback for processing LiDAR scan data."""
        total_samples = len(msg.ranges)  # Total number of LiDAR samples (1080)

        # Calculate the indices for the desired angle ranges
        angle_min = -135.0  # Start angle of LiDAR in degrees
        angle_max = 135.0   # End angle of LiDAR in degrees
        left_min_angle = -100
        left_max_angle = -80
        right_min_angle = 80
        right_max_angle = 100

        # Convert the angles to corresponding indices in the scan array
        left_min_index = int(((left_min_angle - angle_min) / (angle_max - angle_min)) * total_samples)
        left_max_index = int(((left_max_angle - angle_min) / (angle_max - angle_min)) * total_samples)
        right_min_index = int(((right_min_angle - angle_min) / (angle_max - angle_min)) * total_samples)
        right_max_index = int(((right_max_angle - angle_min) / (angle_max - angle_min)) * total_samples)

        # Extract the LiDAR samples for the left and right ranges based on indices
        left_ranges = np.array(msg.ranges[left_min_index:left_max_index])
        right_ranges = np.array(msg.ranges[right_min_index:right_max_index])

        # Merge left and right ranges
        self.filtered_scan_data = np.concatenate((left_ranges, right_ranges))
        self.filtered_scan_angles = np.concatenate((
            np.linspace(left_min_angle, left_max_angle, len(left_ranges)),
            np.linspace(right_min_angle, right_max_angle, len(right_ranges))
        ))

    def pose_callback(self, msg):
        """Callback for processing robot's pose data."""
        if len(msg.data) >= 3:
            self.position = (msg.data[0], msg.data[1])  # x, y position
            self.orientation = math.radians(msg.data[2])  # Convert yaw from degrees to radians

            # Store initial position and orientation
            if not self.initial_pose_set:
                self.initial_position = self.position
                self.initial_pose_set = True

    def update_plot(self):
        """Update the map with new LiDAR data and plot robot's position."""
        if self.filtered_scan_data is None or self.position is None:
            return

        # Convert polar coordinates to Cartesian (x, y)
        ranges = self.filtered_scan_data
        angles = np.radians(self.filtered_scan_angles)  # Convert angles to radians

        # Filter points under 24 meters
        valid_indices = ranges < 1.4
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Convert polar coordinates to Cartesian (x, y)
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)

        # Transform local coordinates to global coordinates
        robot_x, robot_y = self.position
        x_global = robot_x + x_local * np.cos(self.orientation) - y_local * np.sin(self.orientation)
        y_global = robot_y + x_local * np.sin(self.orientation) + y_local * np.cos(self.orientation)

        # Append new LiDAR points to the map if they are at least 0.1 meters away from the last plotted point
        for xi, yi in zip(x_global, y_global):
            if self.lidar_points_x and self.lidar_points_y:
                # Check the distance from the last plotted point
                last_x = self.lidar_points_x[-1]
                last_y = self.lidar_points_y[-1]
                if self.distance(xi, yi, last_x, last_y) < 0.1:
                    continue  # Skip if the new point is too close to the last plotted point
            
            self.lidar_points_x.append(xi)
            self.lidar_points_y.append(yi)

        # Update the scatter plot with the new LiDAR points
        self.scat.set_offsets(np.c_[self.lidar_points_x, self.lidar_points_y])

        # Update the robot's position plot and orientation arrow
        self.position_plot.set_data(robot_x, robot_y)

        if self.arrow:
            self.arrow.remove()

        # Add arrow to indicate the robot's orientation
        arrow_length = 1.0  # Length of the arrow
        self.arrow = Arrow(robot_x, robot_y, arrow_length * np.cos(self.orientation), arrow_length * np.sin(self.orientation), width=0.5, color='r')
        self.ax.add_patch(self.arrow)

        # Update plot limits dynamically
        min_x = min(np.min(self.lidar_points_x), robot_x) - 5
        max_x = max(np.max(self.lidar_points_x), robot_x) + 5
        min_y = min(np.min(self.lidar_points_y), robot_y) - 5
        max_y = max(np.max(self.lidar_points_y), robot_y) + 5
        self.ax.set_xlim(min_x, max_x)
        self.ax.set_ylim(min_y, max_y)

        # Redraw the plot
        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.scat)
        self.ax.draw_artist(self.position_plot)
        self.ax.draw_artist(self.arrow)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

    def distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTrackMapper()

    plt.ion()
    plt.legend()
    plt.show()

    # Initial draw
    node.fig.canvas.draw()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()