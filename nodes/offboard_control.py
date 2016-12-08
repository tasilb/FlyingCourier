#!/usr/bin/python
# Adapted from code located at
# dex.px4.io/ros-mavros-offboard.html
# and https://github.com/ucdart/UAV_Commander/blob/master/TCS/default_offboard.py
import rospy, tf, sys
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()

def callback(msg):
    current_state.armed = msg.armed
    current_state.connected = msg.connected
    current_state.mode = msg.mode
    current_state.guided = msg.guided

if __name__ == "__main__":
    rospy.init_node('offb_node')

    state_sub = rospy.Subscriber('mavros/state', State, callback)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
    
    rate = rospy.Rate(20)
    while not current_state.connected:
        rate.sleep()
        
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if (current_state.mode != 'OFFBOARD') and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            rospy.wait_for_service('mavros/set_mode')
            set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
            mode_resp = set_mode_client(0, 'OFFBOARD')
            if mode_resp.success:
                rospy.loginfo("OFFBOARD ENABLED")
            last_request = rospy.Time.now()
        else:
            if (not current_state.armed) and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                rospy.wait_for_service('mavros/cmd/arming')
                arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                arm_resp = arming_client(True)
                if arm_resp.success:
                    rospy.loginfo("VEHICLE ARMED")
                last_request = rospy.Time.now()
                
        local_pos_pub.publish(pose)
        rate.sleep()

