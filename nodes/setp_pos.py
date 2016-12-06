#! /usr/bin/env python

import rospy
from flyco.setp_timer import SetpointTimer
from geometry_msgs.msg import PoseStamped
import sys

if __name__ == "__main__":
    assert len(sys.argv) > 4
    rospy.init_node("setp_pos", anonymous=False)
    posCmd = PoseStamped()
    posCmd.header.stamp = rospy.Time.now()
    posCmd.pose.position.x = float(sys.argv[1])
    posCmd.pose.position.y = float(sys.argv[2])
    posCmd.pose.position.z = float(sys.argv[3])
    timerLengthInSeconds = float(sys.argv[4])
    timedSetp = SetpointTimer(posCmd, 'pos', timerLengthInSeconds)
    timedSetp.start_timer()
    while not rospy.is_shutdown():
	rospy.spin()
