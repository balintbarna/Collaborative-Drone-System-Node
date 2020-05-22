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
from geometry_msgs.msg import Vector3
import math

from mav import mav

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


def _set_pose(pose, x, y, z):
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header = mavros.setpoint.Header(
        frame_id="att_pose",
        stamp=rospy.Time.now())

def create_setpoint_message(x, y, z):
    setpoint_msg = mavros.setpoint.PoseStamped(
        header=mavros.setpoint.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now()),
    )
    setpoint_msg.pose.position.x = x
    setpoint_msg.pose.position.y = y
    setpoint_msg.pose.position.z = z
    return setpoint_msg

def main():
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(20)
    mav1 = mav("/uav1/mavros")
    mav2 = mav("/uav2/mavros")

    # wait for FCU connection
    mav1.wait_for_connection()
    mav2.wait_for_connection()

    # initialize the setpoint
    message_zero = create_setpoint_message(0,0,0)
    message1 = create_setpoint_message(0, 0, 3)
    message2 = create_setpoint_message(1, 0, 3)

    mav1.set_arming(True)
    mav2.set_arming(True)

    # send 100 setpoints before starting
    # for i in range(0, 50):
    #     mav1.setpoint_local_pub.publish(message1)
    #     mav2.setpoint_local_pub.publish(message2)
    #     rate.sleep()

    mav1.set_mode(0, 'OFFBOARD')
    mav2.set_mode(0, 'OFFBOARD')

    last_request = rospy.Time.now()

    # enter the main loop
    while (True):
        # print "Entered whiled loop"
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

        # message1.pose.position.z = 3 + 2*math.sin(rospy.get_time() * 0.2)
        # mav1.setpoint_local_pub.publish(message1)
        mav1.setpoint_local_pub.publish(message_zero)
        # mav1.setpoint_global_pub.publish(message1)
        # message2.pose.position.z = 3 + 2*math.sin(rospy.get_time() * 0.2)
        # mav2.setpoint_local_pub.publish(message2)
        mav2.setpoint_local_pub.publish(message_zero)
        # mav2.setpoint_global_pub.publish(message2)
        rate.sleep()
    return 0


if __name__ == '__main__':
    main()