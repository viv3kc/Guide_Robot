from geometry_msgs.msg import Twist
import math


class WallFollower(object):
    def __init__(self, wall_distance, max_error, vel_publisher):
        self.wall_distance = wall_distance
        self.max_error = max_error
        self.vel_publisher = vel_publisher

    def wall_following_callback(self, sensor_data):
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

        if range_scanned < self.wall_distance:
            vel_msg.angular.z = -0.8
        else:
            abs_error = math.fabs(left_range_scanned - self.wall_distance)
            if abs_error < self.max_error:
                abs_error = self.max_error

            linear = 0.5 + 0.04 * 1 / abs_error
            vel_msg.linear.x = linear
            vel_msg.angular.z = 1.5 * (left_range_scanned - self.wall_distance)

        print "vel_msg " + str(vel_msg)

        self.vel_publisher.publish(vel_msg)
