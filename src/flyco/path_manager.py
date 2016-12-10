import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from flyco.msg import FlycoStatus, FlycoCmd
from datetime import datetime

class  PathManager:
    def __init__(self, setpointList, endingMode='AUTO.LAND'):
        self._setpoint_list = setpointList
        self._setpoint_index = 0
	self._ending_mode = endingMode
        self._max_norm = .2
        self._flyco_status_sub = rospy.Subscriber("/flyco/main_status", FlycoStatus, self._on_satus, queue_size=1)
        self._flyco_cmd_pub = rospy.Publisher('/flyco/cmd', PoseStamped, queue_size=1)
        self._current_status = None
        self._has_fault = False
        self._flyco_cmd_pub.publish(self.setpointList[0])


    def _on_status(self, status):
        self._current_status = status.status
        
        if status.status == FlycoStatus.STATUS_FAULT:
            self._has_fault = True
        
        if(status.status == FlycoCmd.SETPOINT_POS and not self.has_fault):
            if(np.norm(np.array(self.setpointList[self._setpoint_index].pose.position) - \
                       np.array(status.local_pose.pose.position)) < self.max_norm):
                self._setpoint_index += 1
                flycoCmd = FlycoCmd()
                flycoCmd.cmd = FlycoCmd.CMD_SETPOINT_POS
                flycoCmd.setpoint_pose = self.setpointList[self._setpoint_index] 
                self._flyco_cmd_pub.publish(flycoCmd)
