import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion 
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

def get_dist_btw_poses(poseA:np.ndarray, poseB:np.ndarray):
        ''' Returns 2-d ecludian distance between two poses'''

        distance = np.linalg.norm(poseA[:2] - poseB[:2])
        return distance

def get_orient_to_goal(poseA:np.ndarray, poseB:np.ndarray) -> Quaternion:
    '''Returns angle to be moved to align towards goal pose'''
    
    diff = poseB[:2] - poseA[:2]
    dy = diff[1]
    dx = diff[0]

    angle = np.arctan2(dy,dx)
    angle = quaternion_from_euler(ai=0.0, aj=0.0, ak=angle)
    
    angle_qt = Quaternion()
    angle_qt.x = angle[0]
    angle_qt.y = angle[1]
    angle_qt.z = angle[2]
    angle_qt.w = angle[3]
    
    return angle_qt

def get_orient_error(orientA:Quaternion, orientB:Quaternion):
    ''' Returns angle between two poses'''
    
    q1 = quaternion_to_ndarray(orientA)
    q1 = euler_from_quaternion(q1)

    q2 = quaternion_to_ndarray(orientB)
    q2 = euler_from_quaternion(q2)

    dk = q2[2] - q1[2]

    return dk

def pose_to_ndarray(pose:PoseStamped):
    pose_arr = np.array(
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation]
    )

    return pose_arr

def calculate_quaternion_displacement(current_orientation:Quaternion, disp_angle):
    # Convert the displacement angles to quaternion
    displacement_quaternion = quaternion_from_euler(disp_angle)

    # Perform quaternion multiplication to update the orientation
    transformed_quaternion = quaternion_multiply(current_orientation, displacement_quaternion)

    return transformed_quaternion

def quaternion_to_ndarray(q: Quaternion):

    return np.array([q.x, q.y, q.z, q.w])