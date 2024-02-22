# straight_line_planner_node.py
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from global_planners.pose_utils import get_orient_to_goal
import math

import numpy as np


class StraightLinePlannerNode():
    def __init__(self):
        self.traversed_waypoints = []


    # def generate_straight_line_path(self,start_pose,end_pose,num_waypoints,mode,mostart_pose):
    #     path_msg_copy = Path()

        
    #     if mode==0:

    #         path_msg = Path()
            
    #         path_msg.header.frame_id = 'map'  # Adjust frame_id based on your map frame
    #         print(start_pose.pose.position.x)

    #         waypoints = self.interpolate_poses(start_pose, end_pose, num_waypoints+1)

    #         arr1 = np.array([waypoints[0].pose.position.x,waypoints[0].pose.position.y,0.0])
    #         arr2 = np.array([waypoints[1].pose.position.x,waypoints[1].pose.position.y,0.0])

    #         orien = get_orient_to_goal(arr1,arr2)


    #         # Populate the Path message
    #         for waypoint in waypoints:
    #             waypoint.pose.orientation.x = orien.x
    #             waypoint.pose.orientation.y = orien.y
    #             waypoint.pose.orientation.z = orien.z
    #             waypoint.pose.orientation.w = orien.w
    #             path_msg.poses.append(waypoint)

    #         path_msg_copy.header = path_msg.header
    #         path_msg_copy.poses = path_msg.poses.copy()

    #     if mode==1:

    #         path_msg = Path()
            
    #         path_msg.header.frame_id = 'map'  # Adjust frame_id based on your map frame

    #         waypoints = self.interpolate_poses(start_pose, end_pose, num_waypoints+1)
    #         arr1 = np.array([waypoints[0].pose.position.x,waypoints[0].pose.position.y,0.0])
    #         arr2 = np.array([waypoints[1].pose.position.x,waypoints[1].pose.position.y,0.0])

    #         orien = get_orient_to_goal(arr1,arr2)

    #         # Populate the Path message
    #         for waypoint in waypoints:
    #             waypoint.pose.orientation.x = orien.x
    #             waypoint.pose.orientation.y = orien.y
    #             waypoint.pose.orientation.z = orien.z
    #             waypoint.pose.orientation.w = orien.w
    #             path_msg.poses.append(waypoint)
            
    #         for s in path_msg.poses:
    #             self.traversed_waypoints.append(s)
                
    #         index = self.find_closest_waypoint_index(mostart_pose,self.traversed_waypoints)
    #         # print(mostart_pose.pose.position.x)
    #         twp = self.traversed_waypoints[index]
    #         # print(twp.pose.position.x)

    #         dist = self.distance_calc(mostart_pose,twp)
    #         # print(dist)
    #             # path_msg_copy = Path()

    #         path_msg_copy.header = path_msg.header
    #         path_msg_copy.poses = path_msg.poses.copy()

    #         if dist<0.5:
    #             path_msg_copy.poses = path_msg_copy.poses[index:]


    #     return path_msg_copy
        

    def generate_straight_line_path_init(self,start_pose,end_pose,num_waypoints):
        path_msg_copy = Path()

        path_msg = Path()
        
        path_msg.header.frame_id = 'map'  # Adjust frame_id based on your map frame

        waypoints = self.interpolate_poses(start_pose, end_pose, num_waypoints+1)

        arr1 = np.array([waypoints[0].pose.position.x,waypoints[0].pose.position.y,0.0])
        arr2 = np.array([waypoints[1].pose.position.x,waypoints[1].pose.position.y,0.0])

        orien = get_orient_to_goal(arr1,arr2)


        # Populate the Path message
        for waypoint in waypoints:
            waypoint.pose.orientation.x = orien.x
            waypoint.pose.orientation.y = orien.y
            waypoint.pose.orientation.z = orien.z
            waypoint.pose.orientation.w = orien.w
            path_msg.poses.append(waypoint)

        return path_msg


    def interpolate_poses(self, start_pose, goal_pose, num_waypoints):
        # Interpolate between start and goal poses to generate waypoints
        waypoints = []
        for i in range(num_waypoints):
            alpha = float(i) / num_waypoints
            waypoint = PoseStamped()
            waypoint.header.frame_id = start_pose.header.frame_id
            waypoint.pose.position.x = (1 - alpha) * start_pose.pose.position.x + alpha * goal_pose.pose.position.x
            waypoint.pose.position.y = (1 - alpha) * start_pose.pose.position.y + alpha * goal_pose.pose.position.y
            waypoint.pose.orientation = start_pose.pose.orientation  # No interpolation for orientation
            waypoints.append(waypoint)
        return waypoints
    
    def distance_calc(self,pose1,pose2):
        distance = math.sqrt(
                (pose1.pose.position.x - pose2.pose.position.x) ** 2 +
                (pose1.pose.position.y - pose2.pose.position.y) ** 2
            )

        return distance
    
    def find_closest_waypoint_index(self, current_position, posestd_array ):
        closest_distance = float('inf')
        closest_waypoint_index = None
        if not posestd_array:

            return None
        for i, waypoint in enumerate(posestd_array, start=1):
            distance = math.sqrt(
                (current_position.pose.position.x - waypoint.pose.position.x) ** 2 +
                (current_position.pose.position.y - waypoint.pose.position.y) ** 2
            )

            if distance < closest_distance:
                closest_distance = distance
                closest_waypoint_index = i 

        return closest_waypoint_index - 1


