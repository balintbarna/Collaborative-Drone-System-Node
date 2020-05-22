#!/usr/bin/env python

# import base libs
import numpy as np
import sys
import signal
import math

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose

# import files in package
from mav import Mav
from state_machine import StateMachine

class MainNode():
    def __init__(self):
        rospy.init_node('default_offboard', anonymous=True)
        self.rate = rospy.Rate(20)
        self.mav1 = Mav("uav1/mavros")
        self.mav2 = Mav("uav2/mavros")
        self.sm = StateMachine()
        self.sm.set_params((self.mav1, self.mav2))

        self._command_sub = rospy.Subscriber("pose_command", String, self._pose_command_callback)

        # wait for FCU connection
        self.mav1.wait_for_connection()
        self.mav2.wait_for_connection()

        self.last_request = rospy.Time.now() - rospy.Duration(5.0)

        self.sm.set_current_state(self.sm.States.TAKE_OFF)
    
    def loop(self):
        # enter the main loop
        while (True):
            # print "Entered whiled loop"
            self.arm_and_set_mode()
            self.sm.execute()
            self.rate.sleep()

    def arm_and_set_mode(self):
        if rospy.Time.now() - self.last_request > rospy.Duration(5.0):
            if self.mav1.UAV_state.mode != "OFFBOARD":
                self.mav1.set_mode(0, 'OFFBOARD')
                print("enabling offboard mode")
                self.last_request = rospy.Time.now()
            if not self.mav1.UAV_state.armed:
                if self.mav1.set_arming(True):
                    print("Vehicle armed")
                self.last_request = rospy.Time.now()
            if self.mav2.UAV_state.mode != "OFFBOARD":
                self.mav2.set_mode(0, 'OFFBOARD')
                print("enabling offboard mode")
                self.last_request = rospy.Time.now()
            if not self.mav2.UAV_state.armed:
                if self.mav2.set_arming(True):
                    print("Vehicle armed")
                self.last_request = rospy.Time.now()
        
    def _pose_command_callback(self, topic = String()):
        text = topic.data
        textarr = np.array(text.split(";"))
        arr = textarr.astype(np.float)
        
        pose = Mav.create_setpoint_message_xyz_yaw(arr[0], arr[1], arr[2], arr[3])
        self.mav1.set_target_pose(pose)
        self.mav2.set_target_pose(pose)
        self.sm.set_next_state(self.sm.States.IDLE)
        self.sm.set_current_state(self.sm.States.WAITING_TO_ARRIVE)
        pass


def main():
    node = MainNode()
    node.loop()

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()