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

from mav import mav
from state_machine import StateMachine

def main():
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(20)
    mav1 = mav("uav1/mavros")
    mav2 = mav("uav2/mavros")
    sm = StateMachine()
    sm.set_params((mav1, mav2))

    # wait for FCU connection
    mav1.wait_for_connection()
    mav2.wait_for_connection()

    mav1.set_arming(True)
    mav2.set_arming(True)

    last_request = rospy.Time.now() - rospy.Duration(5.0)

    sm.set_current_state(sm.States.INITIAL_POSITIONS)
    sm.set_next_state(sm.States.MASTER_TO_SKY)
    # enter the main loop
    while (True):
        # print "Entered whiled loop"
        arm_and_set_mode(mav1, mav2, last_request)
        sm.execute()
        rate.sleep()
    return 0

def arm_and_set_mode(mav1, mav2, last_request):
    if (mav1.UAV_state.mode != "OFFBOARD" and
            (rospy.Time.now() - last_request > rospy.Duration(5.0))):
        mav1.set_mode(0, 'OFFBOARD')
        print("enabling offboard mode")
        last_request = rospy.Time.now()
    else:
        if (not mav1.UAV_state.armed and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if (mav1.set_arming(True)):
                print("Vehicle armed")
            last_request = rospy.Time.now()
    if (mav2.UAV_state.mode != "OFFBOARD" and
            (rospy.Time.now() - last_request > rospy.Duration(5.0))):
        mav2.set_mode(0, 'OFFBOARD')
        print("enabling offboard mode")
        last_request = rospy.Time.now()
    else:
        if (not mav2.UAV_state.armed and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if (mav2.set_arming(True)):
                print("Vehicle armed")
            last_request = rospy.Time.now()

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()