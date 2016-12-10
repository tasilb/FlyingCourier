#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from flyco.msg import FlycoStatus, FlycoCmd
from mavros_msgs.srv import SetMode
import numpy as np

class IndoorSafetyMonitor:
    def __init__(self, x1, y1, z1, x2, y2, z2, failsafeMode="AUTO.LAND"):
	self._max_differential = 2
        self._current_differential = 0
        self._alpha = .5 #differential smoothing
	self._failsafe_mode = failsafeMode
	self._corner1 = np.array([x1, y1, z1])
	self._corner2 = np.array([x2, y2, z2])
	self._current_mode = None
	self._last_position = None

        self._flyco_status_sub = rospy.Subscriber("/flyco/main_status", FlycoStatus, self._on_status)
        self._flyco_cmd_pub = rospy.Publisher("/flyco/cmd", FlycoCmd, queue_size=1)
	rospy.wait_for_service("/mavros/set_mode")
	self._set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
	self._rate = rospy.Rate(10)
        rospy.loginfo("[IndoorSafetyMonitor] IndoorSafetyMonitor initialized!")

    def _on_status(self, msg):
	self._current_mode = msg.mavros_state.mode
	msgPos = msg.local_pose.pose.position
	currentPosition = np.array([msgPos.x, msgPos.y, msgPos.z, msg.header.stamp])
	self._validate_position_safety(currentPosition)
	self._last_position = currentPosition

    def _validate_position_safety(self, currentPosition):
	positionOk = self._in_position_bounds(currentPosition) and self._in_differential_bounds(currentPosition)
	if not positionOk:
	    self._enter_failsafe()

    def _in_position_bounds(self, position):
        positionInBounds = np.sum(position[:3] >= self._corner1) + \
                           np.sum(position[:3] <= self._corner2) == 6
	if not positionInBounds:
            rospy.loginfo("[IndoorSafetyMonitor] Local position is outside of safety area: {0} corner one: {1} corner 2: {2}."\
                      .format(position[:3], self._corner1, self._corner2))
	return positionInBounds

    def _in_differential_bounds(self, position):
	if self._last_position is not None:

            rawDifferential = (position[:3] - self._last_position[:3]) / (position[3] - self.last_position[3])
            self._current_differential = self._alpha * self._current_differential + (1 - self._alpha) * rawDifferential
	    validDifferentials = abs(self.current_differential) < self._max_differential
	    differentialInBounds = np.sum(validDifferentials) == 3
	    
            if not differentialInBounds:
                rospy.loginfo("[IndoorSafetyMonitor] Detected excessive position differential: {0} maximum differential: {1}"\
                          .format(self.current_differential, self._max_differential))
	    return differentialInBounds
	else:
	    return True

    def _enter_failsafe(self):
	rospy.loginfo("[IndoorSafetyMonitor] Entering failsafe mode {}".format(self._failsafe_mode))
        cmd = FlycoCmd()
        cmd.cmd = FlycoCmd.CMD_FAILSAFE
        self._flyco_cmd_pub.publish(cmd)
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
