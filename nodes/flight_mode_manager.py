#! /usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from flyco.msg import FlycoStatus

class FlightModeManager:
    def __init__(self):
        self._flyco_status_sub = rospy.Subscriber('/flyco/main_status', FlycoStatus, self._on_state, queue_size=1)
        self._current_mode = None
        self._desired_mode = None
        rospy.wait_for_service('/mavros/set_mode')
        self._set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._resend_rate = rospy.Rate(10)

    def _on_state(self, msg):
        self._current_mode = msg.mavros_state.mode
        currentStatus = msg.status
        if currentStatus == FlycoStatus.STATUS_FAULT or FlycoStatus.STATUS_LANDING:
            self._desired_mode = "AUTO.LAND"
        elif currentStatus == FlycoStatus.STATUS_SETPOINT_NAV:
            self._desired_mode = "OFFBOARD"
        elif currentStatus == FlycoStatus.ACTUATION:
            self._desired_mode = "POSCTL"
        self._mavros_set_state()

    def _mavros_set_state(self):
	while self._current_mode != self._desired_mode:
            rospy.log("[FlightModeManager] Requesting mode switch ({} to {})".format(self._current_mode, self._desired_mode))
	    self._set_mode_client(custom_mode=self._desired_mode)
	    self._resend_rate.sleep()

    def run(self):
       rospy.spin()

if __name__ == "__main__":
    rospy.init_node("flyco_flight_mode_manager", anonymous=False)
    flightModeManager = FlightModeManager()
    flightModeManager.run()
