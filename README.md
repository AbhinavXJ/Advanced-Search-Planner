# Modular Search and Straight Line Planners for ROS2 Autonomous Rovers

This repository provides custom modular path planning algorithms for autonomous rovers in ROS2. The core functionality includes a **zigzag search planner** for systematic area coverage and a **straight line planner** for direct path generation. Both planners are designed for integration with ROS2 navigation stacks and utilize standard ROS message types.

---

## Features

- **Zigzag (Search) Path Planning:**  
  Generates a systematic zigzag (lawnmower) path for area coverage, ideal for search-and-rescue, exploration, and mapping tasks.

- **Straight Line Path Planning:**  
  Computes a direct path between two poses, useful for simple navigation tasks.

- **Modular & Extensible:**  
  The planners are implemented as Python classes, making them easy to integrate and extend.

- **ROS2 Compatible:**  
  Uses standard ROS2 message types (`nav_msgs/Path`, `geometry_msgs/PoseStamped`).

---

## File Structure

| File                              | Description                                   |
|------------------------------------|-----------------------------------------------|
| `search_planner_node.py`           | Zigzag search path planner implementation     |
| `straight_line_planner_node.py`    | Straight line path planner implementation     |
| `global_planners/pose_utils.py`    | Utility functions for pose/orientation        |

---

## Usage

### 1. Zigzag Search Planner (`search_planner_node.py`)

Implements a modular zigzag (lawnmower) search pattern.  
**Key method:** `generate_path_search_init(...)`

#### Parameters

- `theta`: Angle (degrees) for zigzag orientation
- `R`: Step length for each zigzag segment
- `num_waypoints`: Number of waypoints per segment
- `num_change`: Number of zigzag turns
- `start_pose`: Initial pose (`PoseStamped`)
- `dir_start`: Direction flag (0 or 1)
- `start_str`: Start mode flag (1 for starting with a straight segment)

#### Example Usage

from search_planner_node import SearchPlannerNode

planner = SearchPlannerNode()
path = planner.generate_path_search_init(
theta=45,
R=5.0,
num_waypoints=10,
num_change=6,
start_pose=initial_pose,
dir_start=0,
start_str=1
)


#### Main Functions

- `generate_path_search_init`: Generates the full zigzag path as a `nav_msgs/Path`.
- `interpolate_poses`: Linearly interpolates waypoints between two poses.
- `calc_end_pose`: Calculates the end pose for each zigzag segment.
- `find_closest_waypoint_index`: Finds the closest waypoint to the current position.
- `distance_calc`: Computes Euclidean distance between two poses.
- `euler_from_quaternion`: Converts quaternion orientation to Euler angles.

---

### 2. Straight Line Planner (`straight_line_planner_node.py`)

Generates a straight-line path between two poses.  
**Key method:** `generate_straight_line_path_init(...)`

#### Parameters

- `start_pose`: Starting pose (`PoseStamped`)
- `end_pose`: Goal pose (`PoseStamped`)
- `num_waypoints`: Number of waypoints along the line

#### Example Usage

from straight_line_planner_node import StraightLinePlannerNode

planner = StraightLinePlannerNode()
path = planner.generate_straight_line_path_init(
start_pose=initial_pose,
end_pose=goal_pose,
num_waypoints=20
)


#### Main Functions

- `generate_straight_line_path_init`: Returns a `nav_msgs/Path` with interpolated waypoints.
- `interpolate_poses`: Interpolates waypoints between start and goal pose.
- `distance_calc`: Computes distance between two poses.
- `find_closest_waypoint_index`: Finds the index of the closest waypoint.

---

## Dependencies

- ROS2 (tested with Humble/Foxy)
- `numpy`
- `nav_msgs`
- `geometry_msgs`

---

## Integration

1. Place the planner files in your ROS2 workspace (e.g., `src/your_package/`).
2. Import and use the planner classes in your ROS2 nodes.
3. Publish the generated `nav_msgs/Path` to a ROS topic for visualization or navigation.

---

## Example: Publishing a Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathPublisher(Node):
def init(self):
super().init('path_publisher')
self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
# Initialize planner and generate path as shown above
# self.publisher_.publish(path)

def main(args=None):
rclpy.init(args=args)
node = PathPublisher()
rclpy.spin(node)
rclpy.shutdown()


---

## Notes

- The planners assume a 2D plane (x, y), with orientation handled via quaternions.
- Adjust `frame_id` as needed to match your map or odometry frame.
- For full functionality, ensure `global_planners/pose_utils.py` contains the required orientation utility.

---

## License

MIT License

---

## Contribution

Contributions are welcome! Please open issues or submit pull requests for improvements and bug fixes.

---

## Author

*Abhinav Jha*  
*https://github.com/AbhinavXJ/*

---

## Acknowledgments

- ROS2 community
- Open source contributors

---

**Copy and paste this README into your GitHub repository for clear documentation of your custom modular search and straight line planners for ROS2 autonomous rovers.**
