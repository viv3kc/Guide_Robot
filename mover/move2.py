#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import random

turn_right = False
already_turning = False


# Author: Tjark Siewers
# I have taken some code from the robotics 2017

def callback(sensor_data):
    global velocity_publisher
    global already_turning
    # sensor_data (LaserScan data type) has the laser scanner data
    # vel_msg (Twist data type) created to control the base robot

    vel_msg = Twist()

    forward_index1 = len(sensor_data.ranges) / 3
    forward_index2 = 2 * len(sensor_data.ranges) / 3

    range_scanned = min(sensor_data.ranges[(len(sensor_data.ranges) / 2): forward_index2])
    right_range_scanned = min(sensor_data.ranges[0: forward_index1])
    left_range_scanned = min(sensor_data.ranges[forward_index2: len(sensor_data.ranges)])

    print "Minimum range value from 60 to 120 degrees angel: " + str(range_scanned)
    print "range_scanned " + str(range_scanned)
    print 'Left range: ' + str(left_range_scanned)
    print 'Right range: ' + str(right_range_scanned)

    if range_scanned < 1.00:
        vel_msg.angular.z = -0.8
    else:
        abs_error = math.fabs(left_range_scanned - 1.00)
        if abs_error < 0.1:
            abs_error = 0.1

        linear = 0.5 + 0.04 * 1 / abs_error
        vel_msg.linear.x = linear
        vel_msg.angular.z = 1.5 * (left_range_scanned - 1.00)

    print "vel_msg " + str(vel_msg)

    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    # Starts a new node
    global velocity_publisher
    print "Starting mover"
    rospy.init_node('mover', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('base_scan', LaserScan, callback)
    print "Waiting for Caller"
    rospy.spin()