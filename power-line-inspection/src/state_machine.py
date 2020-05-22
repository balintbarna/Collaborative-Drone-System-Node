from enum import Enum

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose

class StateMachine():
    class States(Enum):
        IDLE = "idle"
        WAITING_TO_ARRIVE = "wait_arrive"
        INITIAL_POSITIONS = "initial_pos"
        MASTER_TO_SKY = "master_to_sky"
        SLAVE_UNDER_WIRE = "slave_under_wire"

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
        
        elif cur == self.States.INITIAL_POSITIONS:
            target = Point(0, 0, 3)
            self._mav1.set_target_pos(target)
            self._mav2.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)

        elif cur == self.States.MASTER_TO_SKY:
            target = Point(0, 0, 10)
            self._mav1.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.SLAVE_UNDER_WIRE)

        elif cur == self.States.SLAVE_UNDER_WIRE:
            pass
