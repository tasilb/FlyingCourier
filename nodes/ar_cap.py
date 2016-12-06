#! /usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
import flyco.exp_quat_func as eqf
from threading import Lock

class ARPoseEstimator:
    def __init__(self, h, w):
	self._ar_marker_size_in_m = 20.0 / 1000.0
	self._ar_marker_center_offset = self._ar_marker_size_in_m / 2.0
	self._zero_rotation = np.array([0, 0, 0, 1])
	self._numTags = h, w
	self._coord2Tag = np.arange(self._numTags).reshape((h, w))
	self._lastKnownRbt = np.zeros((self._numTags, 4, 4))
	self._lastRbtUpdateTime = np.zeros(self._numTags)
	self._rbt_status_lock = Lock()

	# cache RBT from usb_cam to fcu
	tfListener = tf.TransformListener()
	tfListener.waitForTransform('fcu', 'usb_cam', rospy.Time(), rospy.Duration(10.0))
	cam2fcuTrans, cam2fcuRot = tfListener.lookupTransform('fcu', 'usb_cam', rospy.Time())
	self._cam2fcuRBT = self._return_rbt(cam2fcuTrans, cam2fcuRot)

	# cache grid RBTs as matrix exponentials
	self._tagRBTs = np.zeros((self._numTags, 4, 4))
	print("Building RBT array for {} markers ({}, {})".format(numTags, h, w))
	for i in np.arange(h):
	    for j in np.arange(w):
		transX = (self._ar_marker_size_in_m * j) + self._ar_marker_center_offset
		transY = (self._ar_marker_size_in_m * i) + self._ar_marker_center_offset
		transZ = 0
		trans = np.array([transX, transY, transZ])
		tagRBT = self._return_rbt(trans, self._zero_rotation)
		self._tagRBTs[self._coord2Tag[i, j], :, :] = tagRBT

	self._tf_sub = rospy.Subscriber('/tf', tf.TFMessage, on_tf, queue_size=1)
	self._mocap_pub = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=1)

    # estimation cycle
    def run(self):
	estimationRate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
	    self._estimate_pose()
	    estimationRate.sleep()


    def _estimate_pose(self):
	estimatedPose = PoseStamped()
	estimatedPose.header.stamp = rospy.Time.now()
	estimatedPose.header.frame_id = 'grid_origin'
	# lock and copy, then unlock
	self._rbt_status_lock.acquire()
	lastRbtUpdateTimeSnapshot = np.copy(self._lastRbtUpdateTime)
	lastKnownRbtSnapshot = np.copy(self._lastKnownRbt)
	self._rbt_status_lock.release()

	# filter indices by timestamp
	timeWindowSize = 0.5
	minStamp = rospy.Time.now() - timeWindowSize
	indicesInWindow = np.where(lastRbtUpdateTimeSnapshot > minStamp)

	# drop outliers and average
	# skipping outlier cleaning for now
	# normalize RBTs?
	rbtsInWindow = lastKnownRbtSnapshot[indicesInWindow]
	# assuming all rotations are same, relies on dropout
	avgTrans = np.average(rbtsInWindow)[:, 3][:3]
	avgRot = tf.transformations.rotation_from_matrix(rbtsInWindow[0])
	avgRot = tf.transformations.quaternion_about_axis(avgRot[0], avgRot[1])
	# publish pose based on origin and regularized RBT
	estimatedPose.pose.position.x = avgTrans[0]
	estimatedPose.pose.position.y = avgTrans[1]
	estimatedPose.pose.position.z = avgTrans[2]
	estimatedPose.pose.orientation.x = avgRot[0]
	estimatedPose.pose.orientation.y = avgRot[1]
	estimatedPose.pose.orientation.z = avgRot[2]
	estimatedPose.pose.orientation.w = avgRot[3]
	self._mocap_pub.publish(estimatedPose)

    # cb on ar_track_alvar tf
    def _on_tf(msg):
    #	convert tf to RBT
	for tfs in msg.transforms:
	    frameName = tfs.child_frame_id
	    if len(frameName) > 10 and frameName[:9] == 'ar_marker_':
		stamp = tfs.header.stamp.to_sec()
		tagId = int(frameName.split('_')[2])
		trans, rot = tfs.transform
		tfRbt = self._return_rbt(trans, rot)
		rbt = tagRBTs[tagId].dot(tfRbt).dot(cam2fcuRBT) 
    #		update tag's latest timestamp and last known RBT
		self._rbt_status_lock.acquire()
		lastRbtUpdateTime[tagId] = stamp
		lastKnownRbt[tagId, :, :] = rbt
		self._rbt_status_lock.release()

    def _return_rbt(trans, rot):
	omega, theta = eqf.quaternion_to_exp(rot)
	return eqf.create_rbt(omega, theta, trans)

if __name__ == '__main__':
    rospy.init_node('ar_cap', anonymous=False)
    poseEstimator = ARPoseEstimator(10, 10)
    poseEstimator.run()
