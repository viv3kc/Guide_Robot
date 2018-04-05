import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib
import util
import math
from tts import TextToSpeech as tts


class GoalSender(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        print 'Got server'

    def send_goal(self, landmark):
        action_goal = MoveBaseActionGoal()

        action_goal.header.stamp = rospy.Time.now()
        action_goal.header.frame_id = 'map'

        action_goal.goal_id.id = landmark.name
        action_goal.goal_id.stamp = rospy.Time.now()

        action_goal.goal.target_pose.header.stamp = rospy.Time.now()
        action_goal.goal.target_pose.header.frame_id = 'map'

        action_goal.goal.target_pose.pose.position.x = landmark.x
        action_goal.goal.target_pose.pose.position.y = landmark.y
        action_goal.goal.target_pose.pose.position.z = 0

        action_goal.goal.target_pose.pose.orientation.w = 1
        action_goal.goal.target_pose.pose.orientation = util.rotate_quaternion(
            action_goal.goal.target_pose.pose.orientation,
            math.radians(landmark.angle)
        )

        self.client.send_goal(action_goal.goal)
