#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

# Based on
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_WALLFOLLOWING


class WallFollower(object):
    def __init__(self):
        self.wall_distance = 0.5  # distance desired to a wall
        self.error = 0  # difference between desired distance and the actual distance
        self.diff_e = 0  # difference between current error and previous one
        self.max_speed = 0.8  # maximum forward speed
        self.direction = -1  # 1 for left wall; -1 for right wall
        self.P = 10  # proportional constant
        self.D = 5  # derivative constant
        self.angle_coefficient = 1  # angle coefficient
        self.angle_min = 0  # angle at which the minimum distance was measured
        self.dist_front = 0  # distance from the closest object in front of the robot
        self.min_distance = 0

        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.handle_laser_scan, queue_size=1)

        # time.sleep(10)
        # print 'unregister'
        # self.laser_subscriber.unregister()

    def handle_laser_scan(self, laser_scan):
        ranges_length = len(laser_scan.ranges)

        start_index = ranges_length * (self.direction + 1) / 4  # start index of the checked range
        end_index = ranges_length * (self.direction + 3) / 4  # end index of the checked range

        self.min_distance = min(laser_scan.ranges[start_index: end_index])  # distance to the closest point
        min_index = laser_scan.ranges.index(self.min_distance)  # index of the corresponding laser reading

        self.angle_min = (min_index - ranges_length / 2) * laser_scan.angle_increment
        self.dist_front = min(laser_scan.ranges[ranges_length / 4: ranges_length * 3 / 4])
        self.diff_e = (self.min_distance - self.wall_distance) - self.error
        self.error = self.min_distance - self.wall_distance
        self.publish_message()

    def publish_message(self):
        twist_msg = Twist()
        print self.min_distance
        twist_msg.angular.z = self.direction * (self.P * self.error + self.D * self.diff_e) + \
            self.angle_coefficient * (self.angle_min - math.pi * self.direction / 2)

        if self.dist_front < self.wall_distance:
            twist_msg.linear.x = 0
        elif self.dist_front < self.wall_distance * 2:
            twist_msg.linear.x = 0.5 * self.max_speed
        elif math.fabs(self.angle_min) > 1.75:
            twist_msg.linear.x = 0.4 * self.max_speed
        else:
            twist_msg.linear.x = self.max_speed

        self. twist_publisher.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('wall_follower')
    node = WallFollower()
    rospy.spin()
