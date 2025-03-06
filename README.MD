# Trajectory Visualization and Storage for AMR Navigation

This ROS package provides tools for visualizing and storing trajectory data during Autonomous Mobile Robot (AMR) navigation. It simplifies the process of tracking, visualizing, and analyzing robot paths by automating trajectory visualization in RViz and facilitating easy storage in various file formats.

## Package Overview

The package consists of two main nodes:

1. **Trajectory Publisher and Saver Node**: Collects and publishes the robot's path as it moves through the environment, with the ability to save trajectory data to files.

2. **Trajectory Reader and Publisher Node**: Reads saved trajectory files and publishes the data for visualization in RViz.

## Features

- Real-time trajectory visualization in RViz
- Trajectory data storage in JSON format
- Custom service for saving trajectory data with specified duration
- Transformation of trajectory data to the odom frame
- Configurable parameters for visualization and storage

## Installation

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- C++14 or higher
- jsoncpp library

### Building the Package

```bash
# Clone the repository into your catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/yourusername/trajectory_visualization.git

# Install dependencies
sudo apt-get install libjsoncpp-dev

# Build the package
cd ~/catkin_ws
catkin_make

# Source the workspace
source ~/catkin_ws/devel/setup.bash
```

## Usage

### Trajectory Publisher and Saver

This node subscribes to odometry data, visualizes the robot's trajectory, and provides a service to save the trajectory to a file.

```bash
# Launch the trajectory publisher and saver node
roslaunch trajectory_visualization trajectory_saver.launch
```

To save the trajectory data to a file, call the service:

```bash
# Save the last 60 seconds of trajectory data to a file
rosservice call /trajectory_saver/save_trajectory "filename: 'trajectory.json' duration: 60.0"
```

### Trajectory Reader and Publisher

This node reads a saved trajectory file and publishes it for visualization in RViz.

```bash
# Launch the trajectory reader and publisher node
roslaunch trajectory_visualization trajectory_visualization.launch
```

## Algorithm Overview

### Trajectory Collection and Storage

1. **Data Collection**:
   - Subscribe to the `/odom` topic to receive robot position updates
   - Store position data in a circular buffer with timestamps
   - Maintain a fixed-size buffer to limit memory usage

2. **Trajectory Visualization**:
   - Convert position data to marker arrays for RViz visualization
   - Create line strips connecting consecutive points
   - Add sphere markers at each position point
   - Publish marker arrays to the `/trajectory_markers` topic

3. **Trajectory Storage**:
   - When the save service is called, extract the requested duration of data
   - Convert position data to the specified format (JSON)
   - Write data to the specified file with proper error handling

### Trajectory Reading and Visualization

1. **File Reading**:
   - Parse the trajectory file (JSON format)
   - Extract position and timestamp data

2. **Transformation**:
   - Transform position data to the odom frame if needed
   - Create marker arrays for visualization

3. **Visualization**:
   - Publish marker arrays to RViz
   - Configure markers with appropriate colors and scales

## Configuration

### Parameters

The nodes can be configured using the following parameters:

- `trajectory_buffer_size`: Maximum number of trajectory points to store (default: 1000)
- `marker_lifetime`: Lifetime of visualization markers in seconds (default: 0.0 - persistent)
- `marker_scale`: Scale of trajectory markers (default: 0.1)
- `publish_rate`: Rate at which to publish visualization markers (default: 10 Hz)
- `trajectory_file`: Default path for trajectory files (default: package_path/data/trajectory.json)

### Launch Files

- `trajectory_saver.launch`: Launches the trajectory publisher and saver node
- `trajectory_visualization.launch`: Launches the trajectory reader and publisher node with RViz

## Service Definition

The package defines a custom service for saving trajectory data:

```
string filename  # Name of the file to save trajectory data
float64 duration # Duration in seconds to save (from current time backwards)
---
bool success     # Whether the operation was successful
string message   # Status message
```

## Code Structure

```
trajectory_visualization/
├── CMakeLists.txt
├── data/
│   └── trajectory.json
├── include/
│   └── trajectory_visualization/
├── launch/
│   ├── trajectory_saver.launch
│   └── trajectory_visualization.launch
├── package.xml
├── rviz/
│   └── trajectory_config.rviz
├── src/
│   ├── trajectory_reader.cpp
│   └── trajectory_saver.cpp
└── srv/
    └── SaveTrajectory.srv
```

## Optimization Techniques

1. **Circular Buffer**: Uses a fixed-size circular buffer to limit memory usage while maintaining the most recent trajectory data.

2. **Efficient Marker Updates**: Updates only changed markers to reduce computational overhead.

3. **Batch Processing**: Processes trajectory data in batches to improve performance.

4. **Parameter Tuning**: Configurable parameters allow for optimization based on specific use cases.

## Troubleshooting

### Common Issues

1. **Node not found**: Ensure the package is properly built and sourced.
   ```bash
   cd ~/catkin_ws && catkin_make
   source ~/catkin_ws/devel/setup.bash
   ```

2. **No trajectory visualization**: Check that RViz is configured to display marker arrays on the correct topic.

3. **Service call fails**: Verify that the trajectory saver node is running and the service name is correct.

4. **File not found**: Ensure the specified file path is accessible and writable.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS community for providing the framework and tools
- Turtlebot3 project for AMR functionality references

## Contact

For questions or support, please contact [your-email@example.com](mailto:your-email@example.com).