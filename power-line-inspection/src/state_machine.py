from enum import Enum

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
from message_tools import *

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

    def __init__(self):
        self._current_value = self.States.IDLE
        self._next_value = self.States.IDLE
    
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
            self.set_next_state(self.States.IDLE)

        elif cur == self.States.ALIGN_WIRE:
            target = create_setpoint_message_xyz_yaw(-35.5, 30, 13.5, 1)
            self._mav1.set_target_pose(target)
            self._mav2.set_target_pose(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.IDLE)
