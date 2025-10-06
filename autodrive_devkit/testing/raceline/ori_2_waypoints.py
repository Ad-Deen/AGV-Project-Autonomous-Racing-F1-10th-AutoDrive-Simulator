import json
import math

def load_and_transform_waypoints(json_file_path, output_file_path):
    """
    Load waypoints from a JSON file, reverse-transform them from grid-based coordinates back to real-world coordinates,
    and compute the orientation based on consecutive points. The result is saved to a new JSON file.

    Parameters:
        json_file_path (str): Path to the JSON file with the waypoints.
        output_file_path (str): Path to save the output JSON file with positions and orientations.

    Returns:
        None
    """
    try:
        with open(json_file_path, 'r') as f:
            waypoints = json.load(f)  # Load the JSON array of dictionaries

        # Transformation values (from grid to real-world coordinates)
        transform_x, transform_y = 363, 839

        # Reverse the transformation for each waypoint
        transformed_waypoints = [
            ((waypoint["x"] - transform_x) / 100, (waypoint["y"] - transform_y) / 100) for waypoint in waypoints
        ]

        # Initialize list to store positions and orientations
        orientations = []

        # Set the initial orientation as 4.71 radians
        initial_orientation = 4.71
        orientations.append({
            "x": transformed_waypoints[0][0],
            "y": transformed_waypoints[0][1],
            "orientation": initial_orientation
        })

        # Function to normalize angle between 0 and 2π
        def normalize_angle(angle):
            return angle % (2 * math.pi)

        # Calculate the orientation between consecutive waypoints
        for i in range(1, len(transformed_waypoints)):
            p0 = transformed_waypoints[i - 1]
            p1 = transformed_waypoints[i]
            
            # Calculate the orientation (theta) using atan2
            orientation = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            
            # Normalize the orientation to be within the range [0, 2π]
            orientation = normalize_angle(orientation)

            # Append the position and orientation to the list
            orientations.append({
                "x": p1[0],
                "y": p1[1],
                "orientation": orientation
            })

        # Print the transformed waypoints with orientations
        for waypoint in orientations:
            print(f"Position x: {waypoint['x']:.2f}, y: {waypoint['y']:.2f}, Orientation: {waypoint['orientation']:.2f} rad")

        # Save the results to a new JSON file
        with open(output_file_path, 'w') as outfile:
            json.dump(orientations, outfile, indent=4)

        print(f"Transformed waypoints with orientations saved to {output_file_path}")

    except FileNotFoundError:
        print(f"Error: File not found: {json_file_path}")
    except json.JSONDecodeError:
        print(f"Error: Failed to decode JSON file: {json_file_path}")

if __name__ == '__main__':
    # Path to the input JSON file
    json_file_path = '/home/deen/testing/raceline/smooth_trajectory.json'
    
    # Path to the output JSON file
    output_file_path = '/home/deen/testing/raceline/waypoint.json'
    
    # Load, transform, and save the positions and orientations to a new JSON file
    load_and_transform_waypoints(json_file_path, output_file_path)
