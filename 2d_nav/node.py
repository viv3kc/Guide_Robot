#!/usr/bin/python

import math
import numpy as np
import time
from tts import TextToSpeech as tts
from wall_follower import WallFollower

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal

import util
from result_handler import ResultHandler
from goal_sender import GoalSender
from command_listener import CommandListener


class MoveBaseTest(object):
    def __init__(self):
        self.is_moving = False
        self.returned_home = False
        self.pick_option_started = True
        self.listening_audio = False

        self.last_published_landmark = None
        self.last_published_goal = None

        self.last_estimated_pose = None
        self.last_particle_cloud = None
        self.certainty_queue = []

        self.tour_started = False
        self.tour_queue = []

        self.confidence = None

        self.result_handler = ResultHandler(self)
        self.goal_sender = GoalSender()

        self.landmarks = util.init_landmarks()
        # Start location will always be the first line in the file
        self.start_location = self.landmarks.pop(0)

        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.goal_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.wall_follower = WallFollower(1.0, 0.1, self.vel_publisher)

        self.command_listener = CommandListener(self.landmarks, self.handle_command)

        self.goal_subscriber = rospy.Subscriber(
            '/move_base/goal',
            MoveBaseActionGoal,
            self.handle_goal_published,
            queue_size=1
        )

        self.status_subscriber = rospy.Subscriber(
            '/move_base/result',
            MoveBaseActionResult,
            self.result_handler.handle_result,
            queue_size=1
        )

        self.pose_subscriber = rospy.Subscriber(
            '/amcl_pose',
            PoseWithCovarianceStamped,
            self.handle_amcl_pose,
            queue_size=1
        )

        self.particle_cloud_subscriber = rospy.Subscriber(
            '/particlecloud',
            PoseArray,
            self.handle_particle_cloud,
            queue_size=1
        )

        self.laser_subscriber = rospy.Subscriber(
            '/base_scan',
            LaserScan,
            self.handle_laser_scan,
            queue_size=1
        )

        # self.pick_an_option()

    def pick_an_option(self):
        self.pick_option_started = True
        self.print_options()
        input_val = input('Choose an option: ')
        n = len(self.landmarks)
        while input_val < 0 or input_val > n:
            self.print_options()
            input_val = input('Choose an option: ')
        self.pick_option_started = False
        if 0 <= input_val - 1 < n:
            print '\nHeading to destination...'
            self.publish_goal(self.landmarks[input_val - 1])

            # if self.last_published_landmark != self.start_location:
            #     self.await_for_cancel()

    def await_for_cancel(self):
        print 'Robot is moving towards destination. Type "1" and press Enter to cancel...'
        input_val = input('Input: ')
        while input_val != 1 and self.is_moving:
            input_val = input('Input: ')
        self.cancel_goal()
        time.sleep(5)
        print '\nHeading back to the waiting position...'
        self.publish_goal(self.start_location)

    def cancel_goal(self):
        goal_id = GoalID()
        goal_id.id = self.last_published_goal.goal_id.id
        self.goal_cancel_publisher.publish(goal_id)
        self.is_moving = False
        # print '\nGoal canceled...'
        if self.tour_started:
            tts.speak('Tour cancelled')
            self.tour_started = False
        else:
            tts.speak('Goal canceled')
        time.sleep(5)
        # print '\nReturning to waiting position...'
        tts.speak('Returning to waiting position')
        time.sleep(2)
        self.publish_goal(self.start_location)

    def publish_goal(self, landmark):
        self.last_published_landmark = landmark
        self.goal_sender.send_goal(landmark)
        if not landmark == self.start_location:
            if not self.tour_started:
                tts.speak('Heading to destination... Follow me please')

        self.is_moving = True

    def print_options(self):
        print ''
        i = 1
        for landmark in self.landmarks:
            print str(i) + '. ' + landmark.name
            i += 1

    def handle_amcl_pose(self, estimated_pose):
        self.last_estimated_pose = estimated_pose
        if self.last_particle_cloud is not None:
            self.estimate_amcl_confidence()

    def handle_goal_published(self, goal):
        self.last_published_goal = goal

    def estimate_amcl_confidence(self):
        distance_sum = 0
        estim_pose = self.last_estimated_pose.pose.pose
        for pose in self.last_particle_cloud.poses:
            diff_x = (estim_pose.position.x - pose.position.x) ** 2
            diff_y = (estim_pose.position.y - pose.position.y) ** 2
            distance_sum += math.sqrt(diff_x + diff_y)
        avg_dist = distance_sum / len(self.last_particle_cloud.poses)
        confidence = 1 / (1 + avg_dist)

        self.certainty_queue.append(confidence)

        if len(self.certainty_queue) >= 10:
            self.certainty_queue.remove(self.certainty_queue[0])

        self.confidence = np.mean(self.certainty_queue)

    def handle_particle_cloud(self, particle_cloud):
        self.last_particle_cloud = particle_cloud

    def handle_laser_scan(self, laser_data):
        if self.confidence is None or self.confidence < 0.65:
            self.wall_follower.wall_following_callback(laser_data)
        else:
            self.laser_subscriber.unregister()
            tts.speak('I am localized.')
            time.sleep(3)
            tts.speak('Moving to waiting location')
            self.publish_goal(self.start_location)

    def handle_command(self, command_text):
        print '\ncommand: ' + command_text + '\n'

        if len(command_text) > 0:
            if not self.is_moving:
                if self.last_published_landmark != self.start_location or self.last_published_landmark is None:
                    if 'tour' in command_text.lower():
                        self.start_tour()
                    elif not self.tour_started:
                        for landmark in self.landmarks:
                            if landmark.name.lower() in command_text.lower():
                                self.publish_goal(landmark)
                                break
            else:
                if self.last_published_landmark != self.start_location:
                    if 'cancel' in command_text.lower():
                        self.cancel_goal()

    def start_tour(self):
        self.tour_started = True
        self.tour_queue = [landmark for landmark in self.landmarks]
        if len(self.tour_queue) != 0:
            tts.speak('Starting tour. Please follow me!')
            self.publish_tour_landmark()

    def publish_tour_landmark(self):
        if len(self.tour_queue) > 0:
            landmark = self.tour_queue.pop()
            self.publish_goal(landmark)
        else:
            self.tour_started = False
            tts.speak('Thank you for your attention!')
            time.sleep(5)
            tts.speak('Heading to waiting position.')
            self.publish_goal(self.start_location)


if __name__ == '__main__':
    rospy.init_node('robot_guide')
    node = MoveBaseTest()
    rospy.spin()
