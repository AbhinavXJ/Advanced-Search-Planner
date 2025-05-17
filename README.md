##Implementation of a custom search planner and straight line planner for autonomous robots  

A robust path planning system for autonomous rovers and robots that includes both straight-line planning and search pattern capabilities.

Overview
This repository contains Python-based path planning algorithms for autonomous robots operating in a ROS environment. It provides two main planning strategies:

Straight Line Planning - Creates direct paths between two points
Search Pattern Planning - Generates systematic search patterns for area coverage
Both planners generate navigation paths with proper orientation and waypoint management for effective autonomous navigation.

Features
Straight Line Planner
Efficient point-to-point navigation
Automatic orientation calculation along the path
Waypoint interpolation with configurable density
Search Pattern Planner
Configurable search patterns with parameter control
Support for different search directions
Dynamic angle and radius adjustments
Path continuity handling
Common Utilities
Waypoint tracking and management
Distance calculation between poses
Quaternion to Euler angle conversion
Closest waypoint detection
Dependencies
ROS (Robot Operating System)
Python 3.x
NumPy
Navigation stack packages (nav_msgs, geometry_msgs)
Installation
Clone this repository into your ROS workspace:
bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/rover-path-planning.git
Build your workspace:
bash
cd ~/catkin_ws
catkin_make
Source your workspace:
bash
source ~/catkin_ws/devel/setup.bash
Usage
Straight Line Planner
python
from straight_line_planner_node import StraightLinePlannerNode

# Initialize the planner
planner = StraightLinePlannerNode()

# Generate a path between start and end poses
path = planner.generate_straight_line_path_init(start_pose, end_pose, num_waypoints)
Search Pattern Planner
python
from straight_line_planner_node import SearchPlannerNode

# Initialize the planner
search_planner = SearchPlannerNode()

# Generate a search pattern path
# theta: angle in degrees
# R: radius/distance between search lines
# num_waypoints: density of waypoints along each line
# num_change: number of direction changes (legs in the search pattern)
# dir_start: initial direction (0 or 1)
# start_str: whether to start with a straight line (0 or 1)
path = search_planner.generate_path_search_init(
    theta=45,
    R=10.0,
    num_waypoints=20,
    num_change=4,
    start_pose=current_pose,
    dir_start=0,
    start_str=1
)
How It Works
Straight Line Planning
The straight line planner creates paths between two poses by:

Linearly interpolating between start and end positions
Calculating appropriate orientation for each waypoint
Building a ROS-compatible Path message
Search Pattern Planning
The search pattern planner creates systematic coverage patterns by:

Starting from an initial position and orientation
Generating legs of movement at specified angles
Alternating directions based on configuration parameters
Maintaining orientation alignment with movement direction
Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

Fork the repository
Create your feature branch (git checkout -b feature/amazing-feature)
Commit your changes (git commit -m 'Add some amazing feature')
Push to the branch (git push origin feature/amazing-feature)
Open a Pull Request
License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgments
ROS community for navigation stack components
Contributors to the global_planners package
