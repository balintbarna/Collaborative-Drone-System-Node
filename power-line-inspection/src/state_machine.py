from enum import Enum
from math import sqrt, cos, sin
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
import rospy
from simple_pid import PID

from message_tools import orientation_to_yaw, point_to_arr, arr_to_point

class StateMachine():
    class States(Enum):
        IDLE = "idle"
        WAITING_TO_ARRIVE = "wait_arrive"
        INITIAL_POSITIONS = "initial_pos"
        TAKE_OFF = "takeoff"
        TO_SKY = "master_to_sky"
        CLOSE_TO_WIRE = "close"
        UNDER_WIRE = "slave_under_wire"
        ALIGN_WIRE = "align"
        ALIGN_YAW = "align_yaw"
        ALIGN_POS = "align_pos"

    def __init__(self):
        self._current_value = self.States.IDLE
        self._next_value = self.States.IDLE
        self.pose_error = Pose()

        self._pose_error_sub = rospy.Subscriber("/wire_displacement", PoseStamped, self._pose_error_callback)
        pid_limit = 0.05 # about 3 degrees
        self.yaw_pid = PID(
            Kp=0.1,
            Ki=0.0,
            Kd=0.0,
            setpoint=0,
            sample_time=None,
            output_limits=(-pid_limit, pid_limit))

        pid_limit = 0.05
        self.pos_pid = PID(
            Kp=0.1,
            Ki=0.0,
            Kd=0.0,
            setpoint=0,
            sample_time=None,
            output_limits=(-pid_limit, pid_limit))
        
        self.max_align_error = 0.05
    
    def _pose_error_callback(self, topic = PoseStamped()):
        self.pose_error = topic.pose
    
    def set_params(self, params):
        self._mav1, self._mav2 = params
    
    def set_next_state(self, state):
        self._next_value = state
    
    def set_current_state(self, state):
        print("NEW STATE: " + str(state))
        self._current_value = state
    
    def execute(self):
        cur = self._current_value
        if cur == self.States.IDLE:
            pass

        elif cur == self.States.WAITING_TO_ARRIVE:
            if self._mav1.has_arrived() and self._mav2.has_arrived():
                self.set_current_state(self._next_value)

        elif cur == self.States.TAKE_OFF:
            target = Point(0, 0, 10)
            self._mav1.set_target_pos(target)
            self._mav2.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            # self.set_next_state(self.States.CLOSE_TO_WIRE)
            self.set_next_state(self.States.UNDER_WIRE)

        # elif cur == self.States.CLOSE_TO_WIRE:
        #     target = Point(-35, 30, 12)
        #     self._mav1.set_target_pos(target)
        #     self._mav2.set_target_pos(target)
        #     self.set_current_state(self.States.WAITING_TO_ARRIVE)
        #     self.set_next_state(self.States.UNDER_WIRE)

        elif cur == self.States.UNDER_WIRE:
            target = Point(-35.2, 30, 11)
            self._mav1.set_target_pos(target)
            self._mav2.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.ALIGN_YAW)
            self.pose_error = None
            # self.set_next_state(self.States.IDLE)

        elif cur == self.States.ALIGN_YAW:
            if self.pose_error == None:
                return
            if not self._mav1.has_arrived():
                return
            angle_error = orientation_to_yaw(self.pose_error.orientation)
            self.pose_error = None
            print("angle error "+str(angle_error))
            if abs(angle_error) < self.max_align_error:
                self.set_current_state(self.States.ALIGN_POS)
                print("YAW ALIGNED")
                self.max_align_error -= 0.01
                return
            pid_out = self.yaw_pid(angle_error)
            current_yaw = orientation_to_yaw(self._mav1.current_pose.pose.orientation)
            target_yaw = current_yaw + pid_out
            self._mav1.set_target_yaw(target_yaw)
            self._mav2.set_target_yaw(target_yaw)
        
        elif cur == self.States.ALIGN_POS:
            if self.pose_error == None:
                return
            if not self._mav1.has_arrived():
                return
            err = point_to_arr(self.pose_error.position)
            self.pose_error = None
            magnitude = np.linalg.norm(err)
            print("pos error "+str(magnitude))
            if magnitude < self.max_align_error*20:
                self.set_current_state(self.States.ALIGN_YAW)
                print("POS ALIGNED")
                self.max_align_error -= 0.01
                return
            current_pos = point_to_arr(self._mav1.current_pose.pose.position)
            yaw = orientation_to_yaw(self._mav1.current_pose.pose.orientation)
            err[0] = cos(yaw)*err[0] + sin(yaw)*err[1]
            err[1] = -sin(yaw)*err[0] + cos(yaw)*err[1]
            pid_out = self.pos_pid(magnitude)
            ratio = abs(pid_out / magnitude)
            target = current_pos + ratio * err
            target_point = arr_to_point(target)
            self._mav1.set_target_pos(target_point)
            self._mav2.set_target_pos(target_point)





