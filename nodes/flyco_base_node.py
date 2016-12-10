#! /usr/bin/env python

import rospy
from flyco.msg import FlycoStatus, FlycoCmd
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class FlycoBaseNode:
    def __init__(self):
        self._status = FlycoStatus()
        self._status.status = FlycoStatus.STATUS_INIT
        self._publish_rate = rospy.Rate(50)
        self._mavros_state_sub = rospy.Subscriber("/mavros/state", State, self._on_state)
        self._local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose)
        self._cmd_sub = rospy.Subscriber("/flyco/cmd", FlycoCmd, self._on_cmd)
        self._status_pub = rospy.Publisher("/flyco/main_status", FlycoStatus, queue_size=1)
        rospy.loginfo("[FlycoBase] FlycoBase initialized!")

    def run(self):
        while not rospy.is_shutdown():
            self._status_pub.publish(self._status)
            self._publish_rate.sleep()

    def _on_state(self, msg):
        self._status.mavros_state = msg

    def _on_pose(self, msg):
        self._status.local_pose = msg

    def _on_cmd(self, msg):
        cmdType = msg.cmd
        if self._status.status == FlycoStatus.STATUS_FAULT:
	    if cmdType != FlycoCmd.CMD_FAILSAFE:
		rospy.loginfo("[FlycoBase] Currently in fault state, command rejected.")
            return
        elif cmdType == FlycoCmd.CMD_FAILSAFE:
            rospy.loginfo("[FlycoBase] Entering fault mode.")
            self._status.status = FlycoStatus.STATUS_FAULT
        elif cmdType == FlycoCmd.CMD_LAND:
            rospy.loginfo("[FlycoBase] Entering landing mode.")
            self._status.status = FlycoStatus.STATUS_LANDING
        elif cmdType == FlycoCmd.CMD_SETPOINT_POS:
            rospy.loginfo("[FlycoBase] Entering setpoint navigation mode.")
            rospy.loginfo("[FlycoBase] Desired setpoint: {}".format(msg.setpoint_pose))
            self._status.status = FlycoStatus.STATUS_SETPOINT_NAV
            self._status.setpoint_type = FlycoStatus.SETPOINT_POS
            self._status.setpoint_pose = msg.setpoint_pose
        elif cmdType == FlycoCmd.CMD_ACTUATE:
            rospy.loginfo("[FlycoBase] Entering actuation mode.")
            self._status.status = FlycoStatus.STATUS_ACTUATION
	else:
	    rospy.loginfo("[FlycoBase] Invalid command")

if __name__ == "__main__":
    rospy.init_node("flyco_base_node", anonymous=False)
    flycoBaseNode = FlycoBaseNode()
    flycoBaseNode.run()
