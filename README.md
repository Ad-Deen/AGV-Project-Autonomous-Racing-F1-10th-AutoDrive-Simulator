# AutoDrive DevKit Documentation

**AutoDrive DevKit** is a project for autonomous racing simulation that combines **2D SLAM**, **occupancy grid mapping**, and **optimal path planning** for race tracks in the AutoDrive Simulator environment. It allows building static maps of tracks, computing optimal racing lines for speed laps, and integrating directly with simulator-based agent control.
<p align="center">
  <img src="2D SLAM+Nav.gif" width="600">
</p>
---

## Features

- **2D SLAM & Mapping**
    - The system builds static occupancy grids of race tracks.
    - LIDAR-based mapping and localization are handled via dedicated modules.
    - Post-processing of occupancy grids produces filtered and expanded versions suitable for trajectory planning.
- **Path Planning**
    - Optimal racing lines are generated using **A* algorithms**.
    - Paths can be smoothed using Bezier curves, B-spline, and quintic interpolation methods.
    - Smooth trajectories are stored in JSON files for simulation use.
- **Simulator Integration**
    - Provides ROS 2 bridge scripts for message communication with the simulator.
    - Supports teleoperation for manual testing and debugging.
    - Launch files allow starting the simulator in headless mode or with RViz visualization.
- **Trajectory & Map Visualization**
    - Utilities for plotting occupancy grids, racing lines, and centerlines.
    - Visualization includes labeled track sections, intersections, and optimized racing paths.
- **Modular & Extensible**
    - The repository separates the simulator interface, mapping, path planning, and agent control.
    - New tracks, planning algorithms, or agent controllers can be added easily.

---

## Repository Structure

- **autodrive_f1tenth/**: Simulator bridge scripts and teleoperation tools.
- **launch/**: ROS 2 launch files for starting the simulator in different modes.
- **resource/**: Required resources for simulator packages.
- **rviz/**: RViz configuration files for visualization.
- **testing/**: Contains maps, occupancy grids, path planning scripts, smooth trajectory files, and visualizations.
    - **raceline/**: A* algorithms, smoothing scripts, occupancy grid generation scripts.
    - **racing_agent/**: Mapping, localization, throttle control, and other agent utilities.
    - **Occupancy grid files**: Filtered, expanded, and track occupancy grids.
    - **Trajectory files**: Smooth trajectories and upsampled maps for simulation.
- **setup.py / setup.cfg / package.xml**: Python and ROS package configurations.
- **test/**: Python scripts for style checks and unit tests.

---

## Installation

1. Clone the repository:

```
git clone <repository_url>
cd autodrive_devkit

```

1. Install Python dependencies:

```
pip install -r requirements.txt

```

*(Add simulator-specific dependencies as needed.)*

1. Build the ROS 2 package (if using ROS 2):

```
colcon build
source install/setup.bash

```

---

## 🕹️ AutoDrive Simulator Setup

The **AutoDrive Simulator** is a Unity-based engine that runs independently and communicates with the AutoDrive DevKit.

### Download the Simulator

Download the simulator from this Google Drive link:

[AutoDrive Simulator Download](https://drive.google.com/drive/folders/1WjjEglKLUTzKzqdSDZjVLWvkoVvwC3hb?usp=sharing)

After downloading, extract the folder.

---

### Folder Structure

After extraction, the simulator folder should look like this:

```
autodrive_simulator/
├── AutoDRIVE Simulator.x86_64        # Main executable to start the simulator
├── Data/                             # Contains all game assets and configurations
│   ├── app.info
│   ├── boot.config
│   ├── globalgamemanagers
│   ├── il2cpp_data/
│   ├── Plugins/
│   ├── Resources/
│   └── UnitySubsystems/
├── GameAssembly.so
└── UnityPlayer.so

```

> Note: Do not move or rename the files inside the Data folder. They are required for the simulator engine to function correctly.

---

### Running the Simulator

1. Open a terminal and navigate to the simulator folder:

```bash
cd path/to/autodrive_simulator

```

1. Start the simulator engine:

```bash
./AutoDRIVE\ Simulator.x86_64

```

1. The simulator will launch using Unity, ready to connect with the AutoDrive DevKit.

---

### Folder Placement Recommendation

To ensure smooth integration with the DevKit:

```
root_folder/
├── autodrive_simulator/       # Simulator folder
└── autodrive_devkit/          # DevKit folder

```

- Both folders must reside in the **same root directory**.
- After placement, you can start the simulator and run the ROS 2 / Python DevKit scripts as described in previous sections.

---
## Usage

- **Starting the Simulator**
    - Headless mode:
        
        ```
        ros2 launch autodrive_devkit simulator_bringup_headless.launch.py
        
        ```
        
    - With RViz visualization:
        
        ```
        ros2 launch autodrive_devkit simulator_bringup_rviz.launch.py
        
        ```
        
- **Mapping & SLAM**
    - Run the mapping stack to generate the track occupancy grid:
        
        ```
        ros2 launch testing/launch/map_stack.launch.py
        
        ```
        
- **Path Planning**
    - Generate optimal racing lines using A* algorithms.
    - Smooth trajectories with Bezier, B-spline, or quintic interpolation methods.
    - Trajectories are saved in JSON format for the simulator.
- **Visualization**
    - Plot occupancy grids, racing lines, and centerlines using included scripts.

---

## Output

- **Occupancy Grids**
    - Track occupancy grid, filtered grid, and expanded grid for trajectory planning.
- **Trajectories**
    - Smooth trajectory files (`smooth_trajectory.json`, `updated_centerline.json`, `upsampled_map.json`).
- **Visualizations**
    - Plots showing occupancy grids, racing lines, intersections, and labeled track sections.

---

## Contributing

Contributions, bug reports, and feature requests are welcome. Users are encouraged to submit reproducible issues or pull requests for enhancements.

---

## License

This project is released under the **MIT License**.
