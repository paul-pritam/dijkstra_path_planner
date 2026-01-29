# Dijkstra Planning

A ROS 2 package that implements Dijkstra's algorithm for global path planning in mobile robotics applications.

## Overview

This package provides a path planning node that uses Dijkstra's algorithm to compute optimal paths from a start position to a goal position. It integrates with the ROS 2 navigation stack and can be used for autonomous navigation in both simulated and real environments.

## Features

- **Dijkstra's Algorithm**: Implements the classic Dijkstra shortest path algorithm
- **ROS 2 Integration**: Full integration with ROS 2 services and topics
- **Flexible Grid-Based Planning**: Works with costmaps for obstacle awareness
- **Efficient Path Computation**: Optimized C++ implementation for real-time performance

## Requirements

- ROS 2 (Humble or later)
- C++ 17
- Navigation Messages (`nav_msgs`)
- Geometry Messages (`geometry_msgs`)
- TF2 ROS (`tf2_ros`)

## Installation

### Building from Source

```bash
cd ~/ros2_ws
colcon build --packages-select dijkstra_planning
source install/setup.bash
```

## Package Structure

```
dijkstra_planning/
├── include/dijkstra_planning/
│   └── dijkstra.hpp          # Main Dijkstra algorithm implementation
├── src/
│   └── dijkstra.cpp          # Algorithm implementation details
├── CMakeLists.txt            # Build configuration
├── package.xml               # Package metadata
└── README.md                 # This file
```

## Usage

### Running the Node

```bash
ros2 run dijkstra_planning dijkstra_planner
```

### With Simulation

```bash
ros2 run dijkstra_planning dijkstra_planner --ros-args -p use_sim_time:=true
```

## API

### Services

The package provides the following ROS 2 services for path planning:

- **`plan_path`**: Service to compute a path from start to goal
  - Input: start position (geometry_msgs/PoseStamped), goal position (geometry_msgs/PoseStamped)
  - Output: planned path (nav_msgs/Path)

### Topics

- **Published Topics:**
  - `/path`: The computed path (nav_msgs/Path)
  - `/visualization_marker`: Path visualization for RViz

- **Subscribed Topics:**
  - `/costmap`: The costmap for obstacle detection
  - `/map`: Static map data

## Configuration

The package can be configured through ROS 2 parameters:

```yaml
dijkstra_planning:
  ros__parameters:
    use_sim_time: false
    plan_frequency: 10.0  # Hz
    allow_unknown: false
```

## Example Usage

```bash
# Terminal 1: Launch the map and localization (your workspace uses `mybot`)
ros2 launch mybot map.launch.py

# Terminal 2: Start the Dijkstra planner (use simulation time if needed)
ros2 run dijkstra_planning dijkstra_planner --ros-args -p use_sim_time:=true

```

## Algorithm Details

Dijkstra's algorithm finds the shortest path between nodes in a graph by:

1. Initializing distances to all nodes as infinite, except the start node (distance 0)
2. Selecting the unvisited node with minimum distance
3. Updating distances to neighbor nodes
4. Repeating until all nodes are visited or the goal is reached

**Time Complexity**: O((V + E) log V) where V is vertices and E is edges
**Space Complexity**: O(V)

## Performance Considerations

- Works best with moderate-sized grid maps (< 1000x1000)
- For larger environments, consider using faster heuristic algorithms like A*
- Performance depends on map resolution and obstacle density

## Troubleshooting

### No Path Found
- Check if the goal is reachable (not surrounded by obstacles)
- Verify map is properly published
- Ensure start and goal are within the map bounds

### Poor Performance
- Reduce map resolution if planning is too slow
- Ensure costmap is up-to-date
- Check system resources

## Integration with Navigation Stack

This planner can be integrated with the Nav2 stack:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "dijkstra_planning::DijkstraPlanner"
```

## Contributing

Contributions are welcome! Please ensure code follows the ROS 2 style guide and is properly tested.

## License

Apache License 2.0 - See LICENSE file for details.

## Author

ubuntu <pritampaulwork7@gmail.com>

## References

- Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs"
- ROS 2 Navigation: https://navigation.ros.org/
- ROS 2 Documentation: https://docs.ros.org/
 - Repository: https://github.com/paul-pritam/mybot.git

## Support

For issues, questions, or suggestions, please refer to the ROS 2 community resources or contact the maintainer.
