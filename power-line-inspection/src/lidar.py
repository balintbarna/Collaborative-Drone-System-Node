#!/usr/bin/env python
from math import (degrees, radians, floor, isinf, sin, cos)
import numpy as np

import rospy

from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PoseStamped
from message_tools import create_setpoint_message_xyz_yaw

VU8_100_Conv_Pub = '/LiDAR/VU8_100Deg/laserscan'
VU8_48_Conv_Pub = '/LiDAR/VU8_48Deg/laserscan'

topic_displacement = "/wire_displacement"


class LidarProcessor():
    def __init__(self):
        rospy.init_node('lidar_processing')
        self.front_angle_and_distance = None
        self.back_angle_and_distance = None

        self._lidar48_sub = rospy.Subscriber(VU8_48_Conv_Pub, LaserScan, self.on_lidar_48_data)
        self._lidar100_sub = rospy.Subscriber(VU8_100_Conv_Pub, LaserScan, self.on_lidar_100_data)
        self._displacement_pub = rospy.Publisher(topic_displacement, PoseStamped, queue_size=10)

        self.rate = rospy.Rate(20)
    
    def on_lidar_48_data(self, topic = LaserScan()):
        angle_min = topic.angle_min
        angle_max = topic.angle_max
        assert angle_min == -angle_max
        self.front_angle_and_distance = self.process_lidar_data(topic.ranges, angle_min)
        self.calculate_and_publish_displacement_and_yaw()
        self.front_angle_and_distance = None
        self.back_angle_and_distance = None
    
    def on_lidar_100_data(self, topic = LaserScan()):
        angle_min = topic.angle_min
        angle_max = topic.angle_max
        assert angle_min == -angle_max
        self.back_angle_and_distance = self.process_lidar_data(topic.ranges, angle_max)
    
    def process_lidar_data(self, ranges, limit):
        count = len(ranges)
        angle_value_aggregator = 0
        distance_value_aggregator = 0
        non_inf_count = 0
        for i in range(0, count):
            mult = i/3.5 - 1
            angle_value = mult * limit
            distance_value = ranges[i]
            if not isinf(distance_value):
                angle_value_aggregator += angle_value
                distance_value_aggregator += distance_value
                non_inf_count += 1
        angle_value_average = angle_value_aggregator / non_inf_count if non_inf_count > 0 else np.inf
        distance_value_average = distance_value_aggregator / non_inf_count if non_inf_count > 0 else np.inf
        return angle_value_average, distance_value_average

    def calculate_and_publish_displacement_and_yaw(self):
        if self.front_angle_and_distance == None or self.back_angle_and_distance == None:
            return

        angle_front, distance_front = self.front_angle_and_distance
        angle_back, distance_back = self.back_angle_and_distance
        if isinf(angle_front) or isinf(angle_back):
            return

        angle_error = angle_front - angle_back
        average_angle = (angle_front + angle_back) / 2
        average_distance = (distance_front + distance_back) / 2
        distance_z = cos(average_angle) * average_distance
        distance_xy = sin(average_angle) * average_distance
        distance_x = cos(angle_error) * distance_xy
        distance_y = sin(angle_error) * distance_xy
        msg = create_setpoint_message_xyz_yaw(distance_x, distance_y, distance_z, angle_error)
        self._displacement_pub.publish(msg)


if __name__ == "__main__":
    ls = LidarProcessor()
    rospy.spin()
