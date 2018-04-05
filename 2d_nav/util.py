import math
from landmark import Landmark
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion, Pose
from move_base_msgs.msg import MoveBaseActionGoal
import os.path as path


def rotate_quaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.
    
    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0
 
    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)
 
    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # Multiply new (heading-only) quaternion by the existing (pitch and bank) 
    # quaternion. Order is important! Original orientation is the second 
    # argument rotation which will be applied to the quaternion is the first 
    # argument. 
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions(qa, qb):
    """
    Multiplies two quaternions to give the rotation of qb by qa.
    
    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()
    
    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def get_heading(q):
    """
    Get the robot heading in radians from a Quaternion representation.
    
    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


def init_landmarks():
    # Replace with the absolute path to your landmarks file before launch
    # Path on the robot laptop
    # '/data/private/robot/robotics/src/project/2d_nav/data/map_landmarks.csv'
    my_path = path.abspath(path.dirname(__file__))
    my_path = path.join(my_path, 'data/map_landmarks.csv')
    print my_path
    f = open(
        my_path,
        'r'
    )

    line = f.readline()
    landmarks = []
    while line:
        split = line.split(',')
        landmarks.append(Landmark(split))
        line = f.readline()
    return landmarks


def goal_from_landmark(landmark):
    action_goal = MoveBaseActionGoal()
    action_goal.goal_id.id = landmark.name
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose = Pose()
    goal.pose.position.x = landmark.x
    goal.pose.position.y = landmark.y
    goal.pose.position.z = 0
    goal.pose.orientation.w = 1
    goal.pose.orientation = rotate_quaternion(goal.pose.orientation, math.radians(landmark.angle))

    action_goal.goal.target_pose = goal
    return action_goal
