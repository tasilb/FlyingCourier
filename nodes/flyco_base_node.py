#! /usr/bin/env python

import rospy
from flyco.msg import FlycoStatus, FlycoCmd
from mavros_msgs.msg import State, BatteryState
from geometry_msgs.msg import PoseStamped

class FlycoBaseNode:
    def __init__(self):
        self._status = FlycoStatus()
        self._status.status = FlycoStatus.STATE_INIT
        self._publish_rate = rospy.Rate(50)
        self._mavros_state_sub = rospy.Subscriber("/mavros/state", State, self._on_state)
        self._battery_sub = rospy.Subscriber("/mavros/battery", BatteryState, self._on_battery)
        self._local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose)
        self._cmd_sub = rospy.Subscriber("/flyco/cmd", FlycoCmd, self._on_cmd)
        self._status_pub = rospy.Publisher("/flyco/main_status", FlycoStatus, queue_size=1)
        rospy.log("[FlycoBase] FlycoBase initialized!")

    def run(self):
        while not rospy.is_shutdown():
            self._status_pub.publish(self._status)
            self._publish_rate.sleep()

    def _on_state(self, msg):
        self._status.mavros_state = msg

    def _on_battery(self, msg):
        self._status.battery = msg

    def _on_pose(self, msg):
        self._status.local_pose = msg

    def _on_cmd(self, msg):
        cmdType = msg.cmd
        if self._status.status == FlycoStatus.STATUS_FAULT:
            rospy.log("[FlycoBase] Currently in fault state, command rejected.")
            break
        elif cmdType == FlycoCmd.CMD_FAILSAFE:
            rospy.log("[FlycoBase] Entering fault mode.")
            self._status.status = FlycoStatus.STATUS_FAULT
        elif cmdType == FlycoCmd.CMD_LAND:
            rospy.log("[FlycoBase] Entering landing mode.")
            self._status.status = FlycoStatus.STATUS_LANDING
        elif cmdType == FlycoCmd.CMD_SETPOINT_POS:
            rospy.log("[FlycoBase] Entering entering setpoint navigation mode.")
            self._status.status = FlycoStatus.STATUS_SETPOINT_NAV
            self._status.setpoint_type = FlycoStatus.SETPOINT_POS
            self._status.setpoint_pose = msg.setpoint_pose
        elif cmdType == FlycoCmd.CMD_ACTUATE:
            rospy.log("[FlycoBase] Entering actuation mode.")
            self._status.status = FlycoStatus.STATUS_ACTUATION

if __name__ == "__main__":
    rospy.init_node("flyco_base_node", anonymous=False)
    flycoBaseNode = FlycoBaseNode()
    flycoBaseNode.run()
