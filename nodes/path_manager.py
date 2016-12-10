#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from flyco.msg import FlycoStatus, FlycoCmd, FlycoPath

class PathManager:
    def __init__(self):
        self._max_norm = .2
        self._completed_path = False
        self._ending_cmd_type = None
        self._setpoint_list = None
        self._setpoint_index = None
        self._goal_pose = None
        self._current_status = None
        self._setpoint_cmd = FlycoCmd()
        self._ending_cmd = FlycoCmd()
        self._setpoint_cmd.cmd = FlycoCmd.CMD_SETPOINT_POS
        self._flyco_cmd_pub = rospy.Publisher("/flyco/cmd", FlycoCmd, queue_size=1)
        self._flyco_path_sub = rospy.Subscriber("/flyco/path", FlycoPath, self._on_path)
        self._flyco_status_sub = rospy.Subscriber("/flyco/main_status", FlycoStatus, self._on_status)
	rospy.loginfo("[PathManager] PathManager initialized!")

    def _on_path(self, msg):
        rospy.loginfo("[PathManager] New path received!")
        self._setpoint_list = msg
        self._setpoint_index = 0
        self._goal_pose = self._setpoint_list[self._setpoint_index]
        self._completed_path = False
        self._ending_cmd_type = msg.ending_cmd
        self._ending_cmd.cmd = self._ending_cmd_type

    def _on_status(self, msg):
        self._current_status = msg.status
        if self._has_arrived():
            self._setpoint_index += 1
            if self._setpoint_index >= len(self._setpoint_list):
                rospy.loginfo("[PathManager] Path complete.")
            else:
                self._goal_pose = self._setpoint_list[self._setpoint_index]
                rospy.loginfo("[PathManager] Heading to setpoint {}/{}".format(self._setpoint_index + 1, len(self._setpoint_list)))

    def _run_single_cycle(self):
        if self._current_status is not None and self._setpoint_list is not None:
            fault = self._current_status.status == FlycoStatus.STATUS_FAULT
            if not fault and not self._completed_path and not self._same_goal():
                self._setpoint_cmd.setpoint_pose = self._goal_pose
                self._flyco_cmd_pub.publish(self._setpoint_cmd)
        if self._completed_path:
	    rospy.loginfo("[PathManager] Executing ending command (type {})".format(self._ending_cmd_type))
            self._flyco_cmd_pub.publish(self._ending_cmd)

    def run(self):
        while not rospy.is_shutdown():
            self._run_single_cycle()
            rospy.sleep(0.1)

    def _has_arrived(self):
        if self._current_status is not None and self._setpoint_list is not None:
            desiredPosition = self._goal_pose.pose.position
            desiredPosition = np.array([desiredPosition.x, desiredPosition.y, desiredPosition.z])
            localPosition = self._current_status.setpoint_pose.pose.position
            localPosition = np.array([localPosition.x, localPosition.y, localPosition.z])
            arrived = (np.norm(desiredPosition - localPosition) < self.max_norm)
        else:
            arrived = False

        if arrived:
            rospy.loginfo("[PathManager] Arrived at setpoint!")
        return arrived

    def _same_goal(self):
        statusGoal = self._current_status.setpoint_pose.pose.position
        statusGoal = np.array([statusGoal.x, statusGoal.y, statusGoal.z])
        currentGoal = self._goal_pose.pose.position
        currentGoal = np.array([currentGoal.x, currentGoal.y, currentGoal.z])
        return np.array_equal(statusGoal, currentGoal)

if __name__ == "__main__":
    rospy.init_node("flyco_path_manager", anonymous=False)
    pathManager = PathManager()
    pathManager.run()
