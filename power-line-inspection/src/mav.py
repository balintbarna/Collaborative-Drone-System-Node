#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import sys
import signal
import math

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion

class mav():
    def __init__(self, namespace = "/mavros"):
        self.rate = rospy.Rate(20)
        self.current_pose = Vector3()
        self.UAV_state = mavros_msgs.msg.State()

        mavros.set_namespace(namespace)

        # setup subscriber
        # /mavros/state
        self.state_sub = rospy.Subscriber(mavros.get_topic('state'),
                                    mavros_msgs.msg.State, self._state_callback)
        # /mavros/local_position/pose
        self.local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
            SP.PoseStamped, self._local_position_callback)
        # /mavros/setpoint_raw/target_local
        self.setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
                                            mavros_msgs.msg.PositionTarget, self._setpoint_position_callback)

        # setup publisher
        # /mavros/setpoint/position/local
        self.setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)
        self.setpoint_global_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'global'), PoseStamped, queue_size=10)

        # setup service
        # /mavros/cmd/arming
        self.set_arming = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), mavros_msgs.srv.CommandBool)
        # /mavros/set_mode
        self.set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)

    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided

    def _setpoint_position_callback(self, topic):
        pass

    def _local_position_callback(self, topic):
        pos = topic.pose.position
        print([pos.x, pos.y, pos.z])
        pass

    def wait_for_connection(self):
        while (not self.UAV_state.connected):
            self.rate.sleep()
