import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from datetime import datetime

class SetpointTimer:
    def __init__(self, setpoint, setpointType, timeToPublishSetpointInSeconds, endingMode='AUTO.LAND', nextSetpointTimer=None):
        self._time_to_publish_setpoint_in_seconds = timeToPublishSetpointInSeconds
        self._setpoint = setpoint
	self._ending_mode = endingMode
        self._mavros_state_sub = rospy.Subscriber('/mavros/state', State, self._on_state, queue_size=1)
        self._current_mode = None

        if setpointType == 'lin_vel':
            self._setp_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        elif setpointType == 'ang_vel':
            self._setp_pub = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        elif setpointType == 'att':
            self._setp_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=1)
        elif setpointType == 'throttle':
            self._setp_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle', Float64, queue_size=1)
        elif setpointType == 'pos':
            self._setp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        else:
            print("Invalid setpoint type {}\nAvailable setpoint types: 'lin_vel', 'ang_vel', 'att', 'throttle', 'pos'".format(setpointType))

        self._setpoint_type = setpointType
        rospy.wait_for_service('/mavros/set_mode')
        self._set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
	self._next_setpoint_timer = nextSetpointTimer

    def _on_state(self, msg):
        self._current_mode = msg.mode

    def set_next_setpoint_timer(self, nextSetpointTimer):
	self._next_setpoint_timer = nextSetpointTimer

    def start_timer(self):
	timerStart = datetime.now()
        while (datetime.now() - timerStart).total_seconds() < self._time_to_publish_setpoint_in_seconds:
            self._setp_pub.publish(self._setpoint)
            if self._current_mode != 'OFFBOARD':
                self._set_mode_client(custom_mode='OFFBOARD')
            rospy.sleep(0.1)
	if self._next_setpoint_timer is not None:
	    self._next_setpoint_timer.start_timer()
	else:
	    while self._current_mode != self._ending_mode:
		self._set_mode_client(custom_mode=self._ending_mode)
		rospy.sleep(0.1)

