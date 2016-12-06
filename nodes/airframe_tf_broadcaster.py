#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped

def broadcast():
    fcuListener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    quaternion = tf.transformations.quaternion_from_euler(0, 0.835, 0)
    while not rospy.is_shutdown():
	try:
	    t = fcuListener.getLatestCommonTime("fcu", "local_origin")
	    br.sendTransform((0, 0.05, -0.05),
			     (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
			     t, "usb_cam", "fcu")
	    rate.sleep()
	except tf.Exception as e:
	    print(e)

if __name__ == "__main__":
    rospy.init_node("airframe_tf_broadcaster", anonymous=False)
    broadcast()
