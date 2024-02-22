# straight_line_planner_node.py
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from global_planners.pose_utils import get_orient_to_goal


class SearchPlannerNode():
    def __init__(self):
        self.traversed_waypoints = []
        self.distance_threshold = 1.0
        self.start = PoseStamped()

    def generate_path_search_init(self,theta,R,num_waypoints,num_change,start_pose,dir_start,start_str): 
        thetarad = theta*math.pi/180
        orinn = self.euler_from_quaternion(start_pose.pose.orientation.x,start_pose.pose.orientation.y,start_pose.pose.orientation.z,start_pose.pose.orientation.w)
        self.psi = orinn[2]
        self.psired = ((self.psi-thetarad)*180/math.pi)
        self.psideg = self.psi*180 / math.pi
        
        self.startx = start_pose.pose.position.x
        self.starty = start_pose.pose.position.y     
        path_msg = Path()
        path_msg.header.frame_id = 'map'        # Adjust frame_id based on your map frame

        self.endstrx = self.startx + R*math.cos(self.psi)
        self.endstry = self.starty + R*math.sin(self.psi)
        end_pose=PoseStamped()
        
        if start_str==1:
            end_pose_str = PoseStamped()
            end_pose_str.pose.position.x = self.endstrx
            end_pose_str.pose.position.y = self.endstry

            waypoints_str = self.interpolate_poses(start_pose,end_pose_str,num_waypoints)

            for waypoint in waypoints_str:

                waypoint.pose.orientation.x = start_pose.pose.orientation.x
                waypoint.pose.orientation.y = start_pose.pose.orientation.y
                waypoint.pose.orientation.z = start_pose.pose.orientation.z
                waypoint.pose.orientation.w = start_pose.pose.orientation.w

                path_msg.poses.append(waypoint)

        i=1
        while i<=num_change:
            

            if start_str==1:
                end_pose = self.calc_end_pose(self.endstrx,self.endstry,i,thetarad,R,dir_start)
                waypoints = self.interpolate_poses(end_pose_str, end_pose, num_waypoints)
            else:
                end_pose = self.calc_end_pose(self.startx,self.starty,i,thetarad,R,dir_start)
                waypoints = self.interpolate_poses(start_pose, end_pose, num_waypoints) 
        
            arr1 = np.array([waypoints[0].pose.position.x,waypoints[0].pose.position.y,0.0])
            arr2 = np.array([waypoints[1].pose.position.x,waypoints[1].pose.position.y,0.0])

            orien = get_orient_to_goal(arr1,arr2)

            for waypoint in waypoints:

                waypoint.pose.orientation.x = orien.x
                waypoint.pose.orientation.y = orien.y
                waypoint.pose.orientation.z = orien.z
                waypoint.pose.orientation.w = orien.w

                path_msg.poses.append(waypoint)
        
            if start_str==1:
                end_pose_str.pose.position.x = end_pose.pose.position.x
                end_pose_str.pose.position.y = end_pose.pose.position.y
            else:
                start_pose.pose.position.x = end_pose.pose.position.x
                start_pose.pose.position.y = end_pose.pose.position.y 
            
            i+=1

        return path_msg
    
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

    def calc_end_pose(self,startx,starty,i,thetarad,R,dir_start):
        end_pose=PoseStamped()
        if dir_start == 0:
            match i%2==0:
                case True:
                    end_pose.pose.position.x = startx + i*R*math.cos(thetarad+self.psi)

                    end_pose.pose.position.y = starty + i*R*math.sin(thetarad + self.psi)
                case False:
                    end_pose.pose.position.x = startx + i*R*math.cos(self.psi-thetarad)
                
                    end_pose.pose.position.y = starty + i*R*math.sin(self.psi-thetarad)
        elif dir_start == 1:
            match i%2==0:
                case True:
                    end_pose.pose.position.x = startx + i*R*math.cos(self.psi- thetarad)

                    end_pose.pose.position.y = starty + i*R*math.sin(self.psi - thetarad)
                case False:
                    end_pose.pose.position.x = startx + i*R*math.cos(self.psi+thetarad)
                
                    end_pose.pose.position.y = starty + i*R*math.sin(self.psi+thetarad)
                
        return end_pose

  
    def interpolate_poses(self, start_pose, goal_pose, num_waypoints):
        # Interpolate between start and goal poses to generate waypoints
        waypoints = []
        for i in range(num_waypoints):
            alpha = float(i) / num_waypoints
            waypoint = PoseStamped()
            waypoint.header.frame_id = start_pose.header.frame_id
            waypoint.pose.position.x = (1 - alpha) * start_pose.pose.position.x + alpha * goal_pose.pose.position.x
            waypoint.pose.position.y = (1 - alpha) * start_pose.pose.position.y + alpha * goal_pose.pose.position.y
            waypoints.append(waypoint)

        return waypoints
    
    def distance_calc(self,pose1,pose2):
        distance = math.sqrt(
                (pose1.pose.position.x - pose2.pose.position.x) ** 2 +
                (pose1.pose.position.y - pose2.pose.position.y) ** 2
            )

        return distance

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
