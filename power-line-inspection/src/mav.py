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

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class mav():
    def __init__(self, namespace = "mavros"):
        self.rate = rospy.Rate(20)
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
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
        # self.setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),mavros_msgs.msg.PositionTarget, self._setpoint_position_callback)

        # setup publisher
        # /mavros/setpoint/position/local
        self.setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

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

    # def _setpoint_position_callback(self, topic):
    #     pass

    def _local_position_callback(self, topic):
        self.current_pose = topic

    def wait_for_connection(self):
        while (not self.UAV_state.connected):
            self.rate.sleep()

    def wait_for_arrival(self):
        while not self.has_arrived():
            self.rate.sleep()
    
    def has_arrived(self):
        sp = self.target_pose.pose.position
        sy = mav.orientation_to_yaw(self.target_pose.pose.orientation)
        cp = self.current_pose.pose.position
        cy = mav.orientation_to_yaw(self.current_pose.pose.orientation)
        maxdist = 0.1 # m
        maxang = 0.05 # 3 deg in rad
        diff = abs(sp.x-cp.x)/maxdist + abs(sp.y-cp.y)/maxdist + abs(sp.z-cp.z)/maxdist + abs(sy - cy)/maxang
        if diff < 1:
            return True
        return False


    def set_target_pose(self, pose = PoseStamped()):
        self.target_pose = pose
        self.setpoint_local_pub.publish(pose)
    
    def set_target_pos(self, pos = Point()):
        yaw = mav.orientation_to_yaw(self.target_pose.pose.orientation)
        pose = mav.create_setpoint_message_pos_yaw(pos, yaw)
        self.set_target_pose(pose)

    def set_target_yaw(self, yaw):
        pos = self.target_pose.pose.position
        pose = mav.create_setpoint_message_pos_yaw(pos, yaw)
        self.set_target_pose(pose)
    
    @staticmethod
    def yaw_to_orientation(yaw):
        quat_tf = quaternion_from_euler(0, 0, yaw)
        ori = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
        return ori
    
    @staticmethod
    def orientation_to_yaw(orientation = Quaternion()):
        quat_tf = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = euler_from_quaternion(quat_tf)[2]
        return yaw

    @staticmethod
    def create_setpoint_message_xyz_yaw(x, y, z, yaw = 0):
        pos = Point(x,y,z)
        return mav.create_setpoint_message_pos_yaw(pos, yaw)

    @staticmethod
    def create_setpoint_message_pos_yaw(pos, yaw):
        ori = mav.yaw_to_orientation(yaw)
        return mav.create_setpoint_message_pos_ori(pos, ori)

    @staticmethod
    def create_setpoint_message_pos_ori(pos = Point(), ori = Quaternion()):
        pose = Pose(pos, ori)
        return mav.create_setpoint_message_pose(pose)
    
    @staticmethod
    def create_setpoint_message_pose(pose = Pose()):
        setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
        )
        setpoint_msg.pose = pose
        return setpoint_msg