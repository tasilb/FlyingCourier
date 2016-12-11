#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from flyco.msg import FlycoStatus, FlycoCmd
from datetime import datetime
import sys

class HoverTester:
    def __init__(self, heightInMeters, timeInSeconds):
        self._height_in_meters = heightInMeters
        self._time_in_seconds = timeInSeconds
        self._status = None
        self._flyco_cmd_pub = rospy.Publisher("/flyco/cmd", FlycoCmd, queue_size=1)
        self._flyco_status_sub = rospy.Subscriber("/flyco/main_status", FlycoStatus, self._on_status)

    def run(self):
        hoverCmd = FlycoCmd()
        hoverPose = PoseStamped()
        hoverPose.pose.x = 0
        hoverPose.pose.y = 0
        hoverPose.pose.z = self._height_in_meters
        hoverCmd.cmd = FlycoCmd.CMD_SETPOINT_POS
        hoverCmd.setpoint_pose = hoverPose

        faultCmd = FlycoCmd()
        faultCmd.cmd = FlycoCmd.CMD_FAILSAFE

        landCmd = FlycoCmd()
        landCmd.cmd = FlycoCmd.CMD_LAND

        while self._status != FlycoStatus.STATUS_SETPOINT_NAV and self._status != FlycoStatus.STATUS_FAULT:
            self._flyco_cmd_pub.publish(hoverCmd)

        startTime = datetime.now()
        while (datetime.now() - timerStart).total_seconds() < self._time_in_seconds:
            rospy.spin()

        while not rospy.is_shutdown():
            if self._status != FlycoStatus.STATUS_LANDING and self._status != FlycoStatus.STATUS_FAULT:
                self._flyco_cmd_pub.publish(landCmd)
            rospy.sleep(0.1)

if __name__ == "__main__":
    assert len(sys.argv) > 2
    height = sys.argv[1]
    time = sys.argv[2]
    tester = HoverTester(height, time)
    tester.run()
