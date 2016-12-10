import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from flyco.msg import FlycoStatus

class FlightModeManager:
    def __init__(self):
        self._flyco_status_sub = rospy.Subscriber('/flyco/main_status', FlycoStatus, self._on_state, queue_size=1)
        self._current_mode = None
        rospy.wait_for_service('/mavros/set_mode')
        self._set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._resend_rate = rospy.Rate(10)

    def _on_state(self, msg):
        self._current_mode = msg.mavros_state.mode


    def mavros_set_state(self, state):
	while self._current_mode != self._ending_mode:
	    self._set_mode_client(custom_mode=self._ending_mode)
	    self._resend_rate.sleep()

