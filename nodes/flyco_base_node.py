#! /usr/bin/python

import rospy
from flyco.ar_tag_follower import ARTagFollower
import sys

class CourierNode():
    def __init__(self, markerFrameList):
        self.arFollower = ARTagFollower(markerFrameList)
    
    def run(self):
        while not rospy.is_shutdown():
            self.arFollower.send_follow_command()        


if __name__ == "__main__":
    rospy.init_node("flyco_base_node")
    rospy.loginfo("flyco_base_node starting")
    cn = CourierNode(sys.argv[1])
    cn.run()
    rospy.loginfo("flyco_base_node finished")
