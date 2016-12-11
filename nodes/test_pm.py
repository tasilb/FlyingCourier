#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from flyco.msg import FlycoPath, FlycoStatus

class PathManagerTester:
    def __init__(self, nx3xyzArr):
        self._current_status = None
        self._current_destination = None
	self._current_pose = None
        self._setpoint_list = []
        for position in nx3xyzArr:
            setpointPose = PoseStamped()
            setpointPose.pose.position.x = position[0]
            setpointPose.pose.position.y = position[1]
            setpointPose.pose.position.z = position[2]
            self._setpoint_list.append(setpointPose)
        
        self._flyco_status_sub = rospy.Subscriber("/flyco/main_status", FlycoStatus, self._on_status)
        self._flyco_path_pub = rospy.Publisher("/flyco/path", FlycoPath, queue_size=1)

        self._path = FlycoPath()
        self._path.path = tuple(self._setpoint_list)
        self._path.ending_cmd = FlycoPath.CMD_ACTUATE

    def _on_status(self, msg):
        self._current_status = msg.status
        self._current_destination = msg.setpoint_pose.pose.position
        self._current_pose = msg.local_pose.pose.position

    def run(self):
	while self._current_status != FlycoStatus.STATUS_SETPOINT_NAV:
	    self._flyco_path_pub.publish(self._path)
        while not rospy.is_shutdown():
            print("[PATH TEST] Base status:")
            print(self._current_status)
            print("[PATH TEST] Goal position:")
            print(self._current_destination)
            print("[PATH TEST] Local position:")
            print(self._current_pose)
	    rospy.sleep(1.0)

if __name__ == "__main__":
    rospy.init_node("test_pm", anonymous=False)
    positions = np.array([0.0, 0.0, 1.0])
    positions = np.vstack((positions, np.array([0.8, 0.0, 1.0])))
    positions = np.vstack((positions, np.array([0.8, -0.8, 1.0])))
    positions = np.vstack((positions, np.array([0.0, -0.8, 1.0])))

    tester = PathManagerTester(positions)
    tester.run()
