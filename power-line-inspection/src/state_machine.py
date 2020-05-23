from enum import Enum

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
from message_tools import orientation_to_yaw
import rospy
from simple_pid import PID

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
        self.yaw_pid = PID()
    
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
            self.set_next_state(self.States.CLOSE_TO_WIRE)

        elif cur == self.States.CLOSE_TO_WIRE:
            target = Point(-30, 30, 10)
            self._mav1.set_target_pos(target)
            self._mav2.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.UNDER_WIRE)

        elif cur == self.States.UNDER_WIRE:
            target = Point(-35.5, 30, 13.5)
            self._mav1.set_target_pos(target)
            self._mav2.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.ALIGN_YAW)
            # self.set_next_state(self.States.IDLE)

        elif cur == self.States.ALIGN_YAW:
            angle_error = orientation_to_yaw(self.pose_error.orientation)
            print("angle error "+str(angle_error))
            pid_out = self.yaw_pid(angle_error)
            current_yaw = orientation_to_yaw(self._mav1.current_pose.pose.orientation)
            target_yaw = current_yaw + pid_out
            self._mav1.set_target_yaw(target_yaw)
            if angle_error < 0.1:
                self.set_current_state(self.States.IDLE)
                print("YAW ALIGNED")

