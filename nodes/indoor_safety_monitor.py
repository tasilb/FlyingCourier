import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
import numpy as np

class IndoorSafetyMonitor:
    def __init__(self, x1, y1, z1, x2, y2, z2, failsafeMode='AUTO.LAND'):
	self._max_differential = 2
	self._failsafe_mode = failsafeMode
	self._corner1 = np.array([x1, y1, z1])
	self._corner2 = np.array([x2, y2, z2])
	self._current_mode = None
	self._last_position = None

	self._position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose)
	self._mavros_state_sub = rospy.Subscriber("/mavros/state", State, self._on_state)
	rospy.wait_for_service('/mavros/set_mode')
	self._set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
	self._rate = rospy.Rate(10)

    def _on_pose(self, msg):
	msgPos = msg.pose.position
	currentPosition = np.array([msgPos.x, msgPos.y, msgPos.z])
	self._validate_position_safety(currentPosition)
	self._last_position = currentPosition
    
    def _on_state(self, msg):
	self._current_mode = msg.mode

    def _validate_position_safety(self, currentPosition):
	positionOk = self._in_position_bounds(currentPosition) and self._in_differential_bounds(currentPosition)
	if not positionOk:
	    self._enter_failsafe()

    def _in_position_bounds(self, position):
	positionInBounds = np.sum(position >= self._corner1) + \
			   np.sum(position <= self._corner2) == 6
	if not positionInBounds:
	    rospy.log("[IndoorSafetyMonitor] Local position is outside of safety area.")
	return positionInBounds

    def _in_differential_bounds(self, position):
	if self._last_position is not None:
	    differentials = np.abs(position - self._last_position)
	    validDifferentials = differentials < self._max_differential
	    differentialInBounds = np.sum(validDifferentials) == 3
	    if not differentialInBounds:
		rospy.log("[IndoorSafetyMonitor] Detected excessive position differential.")
	    return differentialInBounds
	else:
	    return True

    def _enter_failsafe(self):
	rospy.log("[IndoorSafetyMonitor] Entering failsafe mode {}".format(self._failsafe_mode))
	while self._current_mode != self._failsafe_mode:
	    self._set_mode_client(custom_mode=self._failsafe_mode)
	    self._rate.sleep()
	rospy.spin()

    def run(self):
	rospy.spin()

if __name__ == '__main__':
    rospy.init_node("flyco_indoor_safety_monitor", anonymous=False)
    safetyMonitor = IndoorSafetyMonitor(-0.2, -0.2, -0.2, 2.0, 2.0, 2.0)
    safetyMonitor.run()
