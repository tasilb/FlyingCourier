#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from flyco.msg import FlycoStatus

class SetpointManager:
    def __init__(self):
        self._flyco_status_sub = rospy.Subscriber("/flyco/main_status", FlycoStatus, self._on_status, queue_size=1)
        self._setp_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self._setpoint_rate = rospy.Rate(10)
	self._setpoint_type = None
        rospy.loginfo("[SetpointManager] SetpointManager initialized!")
            
    def _on_status(self, msg):
        self._setpoint = msg.setpoint_pose
        self._setpoint_type = msg.setpoint_type

    def run(self):
        while not rospy.is_shutdown():
            if self._setpoint_type is not None and self._setpoint_type == FlycoStatus.STATUS_SETPOINT_NAV:
                self._setp_pos_pub.publish(self._setpoint)
            self._setpoint_rate.sleep()

if __name__ == "__main__":
    rospy.init_node("flyco_setpoint_manager")
    setpointManager = SetpointManager()
    setpointManager.run()
