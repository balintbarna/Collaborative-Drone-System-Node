#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
from mavros_msgs.msg import State, PositionTarget
import mavros_msgs.srv
import sys
import signal
import math

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Mav():
    def __init__(self, namespace = "mavros"):
        self.rate = rospy.Rate(20)
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.target_pose = PoseStamped()
        self.UAV_state = mavros_msgs.msg.State()

        mavros.set_namespace(namespace)

        # setup subscriber
        self._state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self._state_callback)
        self._local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, self._local_position_callback)
        self._local_velocity_sub = rospy.Subscriber(mavros.get_topic('local_position', 'velocity_body'), TwistStamped, self._local_velocity_callback)

        # setup publisher
        self._setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)
        self._setpoint_raw_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'local'), PositionTarget)

        # setup service
        self.set_arming = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), mavros_msgs.srv.CommandBool)
        self.set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)

    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided

    def _local_position_callback(self, topic):
        self.current_pose = topic
        self._publish_target_pose()

    def _local_velocity_callback(self, topic):
        self.current_velocity = topic
    
    def _publish_target_pose(self):
        self._setpoint_local_pub.publish(self.target_pose)
    
    def _publish_target_raw(self):
        message = PositionTarget()
        message.position = self.target_pose.pose.position
        message.yaw = Mav.orientation_to_yaw(self.target_pose.pose.orientation)
        message.velocity = Vector3()
        message.acceleration_or_force = Vector3()
        message.yaw_rate = 0
        self._setpoint_raw_pub.publish(message)

    def wait_for_connection(self):
        while (not self.UAV_state.connected):
            self.rate.sleep()

    def wait_for_arrival(self):
        while not self.has_arrived():
            self.rate.sleep()
    
    def has_arrived(self):
        maxdist = 0.5 # m
        maxang = 0.1 # 6 deg in rad
        max_vel = 0.3 # m/s
        max_angvel = 0.05
        posgood = self.get_pos_error() < maxdist
        anggood = self.get_yaw_error() < maxang
        velgood = self.get_velocity_abs() < max_vel
        angvelgood = self.get_ang_vel_abs() < max_angvel
        if posgood and anggood and velgood and angvelgood:
            return True
        return False
    
    def get_pos_error(self):
        sp = self.target_pose.pose.position
        cp = self.current_pose.pose.position
        x = sp.x-cp.x
        y = sp.y-cp.y
        z = sp.z-cp.z
        dist = math.sqrt(x*x + y*y + z*z)
        return dist
    
    def get_yaw_error(self):
        sy = Mav.orientation_to_yaw(self.target_pose.pose.orientation)
        cy = Mav.orientation_to_yaw(self.current_pose.pose.orientation)
        yaw = abs(sy - cy)
        return yaw
    
    def get_velocity_abs(self):
        vel = self.current_velocity.twist.linear
        x = vel.x
        y = vel.y
        z = vel.z
        norm = math.sqrt(x*x + y*y + z*z)
        return norm
    
    def get_ang_vel_abs(self):
        vel = self.current_velocity.twist.angular
        x = vel.x
        y = vel.y
        z = vel.z
        norm = math.sqrt(x*x + y*y + z*z)
        return norm

    def set_target_pose(self, pose = PoseStamped()):
        self.target_pose = pose
    
    def set_target_pos(self, pos = Point()):
        yaw = Mav.orientation_to_yaw(self.target_pose.pose.orientation)
        pose = Mav.create_setpoint_message_pos_yaw(pos, yaw)
        self.set_target_pose(pose)

    def set_target_yaw(self, yaw):
        pos = self.target_pose.pose.position
        pose = Mav.create_setpoint_message_pos_yaw(pos, yaw)
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
        return Mav.create_setpoint_message_pos_yaw(pos, yaw)

    @staticmethod
    def create_setpoint_message_pos_yaw(pos, yaw):
        ori = Mav.yaw_to_orientation(yaw)
        return Mav.create_setpoint_message_pos_ori(pos, ori)

    @staticmethod
    def create_setpoint_message_pos_ori(pos = Point(), ori = Quaternion()):
        pose = Pose(pos, ori)
        return Mav.create_setpoint_message_pose(pose)
    
    @staticmethod
    def create_setpoint_message_pose(pose = Pose()):
        setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
        )
        setpoint_msg.pose = pose
        return setpoint_msg