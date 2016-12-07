#! /usr/bin/env python

import rospy
import tf
from tf.msg import tfMessage
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
import flyco.exp_quat_func as eqf
from threading import Lock
np.set_printoptions(precision=15, suppress=True)

class ARPoseEstimator:
    def __init__(self, h, w):
	self._ar_marker_size_in_m = 20.0 / 1000.0
	self._ar_marker_center_offset = self._ar_marker_size_in_m / 2.0
	self._zero_rotation = np.array([0, 0, 0, 1])
	self._numTags = h * w
	self._coord2Tag = np.arange(self._numTags).reshape((h, w))
	self._lastKnownRbt = np.zeros((self._numTags, 4, 4))
	self._lastRbtUpdateTime = np.zeros(self._numTags)
	self._rbt_status_lock = Lock()

	# cache RBT from usb_cam to fcu
	self._tfListener = tf.TransformListener()
	self._tfListener.waitForTransform('fcu', 'usb_cam', rospy.Time(), rospy.Duration(10.0))
	cam2fcuTrans, cam2fcuRot = self._tfListener.lookupTransform('fcu', 'usb_cam', rospy.Time())
	self._cam2fcuRBT = self._return_rbt(cam2fcuTrans, cam2fcuRot)

	# cache grid RBTs as matrix exponentials
	self._tagRBTs = np.zeros((self._numTags, 4, 4))
	print("Building RBT array for {} markers ({}, {})".format(self._numTags, h, w))
	for i in np.arange(h):
	    for j in np.arange(w):
		transX = (self._ar_marker_size_in_m * j) + self._ar_marker_center_offset
		transY = (self._ar_marker_size_in_m * i) + self._ar_marker_center_offset
		transZ = 0
		trans = np.array([transX, transY, transZ])
		tagRBT = self._return_rbt(trans, self._zero_rotation)
		self._tagRBTs[self._coord2Tag[i, j], :, :] = tagRBT

	self._tf_sub = rospy.Subscriber('/tf', tfMessage, self._on_tf, queue_size=1)
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
	#self._rbt_status_lock.acquire()
	print("estimator lock")
	lastRbtUpdateTimeSnapshot = np.copy(self._lastRbtUpdateTime)
	lastKnownRbtSnapshot = np.copy(self._lastKnownRbt)
	#self._rbt_status_lock.release()

	# filter indices by timestamp
	timeWindowSize = 10.0
	minStamp = self._tfListener.getLatestCommonTime('fcu', 'usb_cam').to_sec() - timeWindowSize
	indicesInWindow = np.where(lastRbtUpdateTimeSnapshot >= minStamp)[0]
	print("lastRbtUpdateTimeSnapshot.shape: {}".format(lastRbtUpdateTimeSnapshot.shape))
	print("indicesInWindow length: {}".format(len(indicesInWindow)))

	# drop outliers and average
	# skipping outlier cleaning for now
	# normalize RBTs?
	rbtsInWindow = lastKnownRbtSnapshot[indicesInWindow]
	print("minimum time: {}".format(minStamp))
	# assuming all rotations are same, relies on dropout
	if rbtsInWindow.shape[0] > 0:
	    print("estimating pose")
	    avgTrans = np.average(rbtsInWindow, axis=0)[:, 3][:3]
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
	    print("publishing pose for time {}".format(estimatedPose.header.stamp))
	    self._mocap_pub.publish(estimatedPose)
	else:
	    print("no valid rbts found")

    # cb on ar_track_alvar tf
    def _on_tf(self, msg):
    #	convert tf to RBT
	for tfs in msg.transforms:
	    frameName = tfs.child_frame_id
	    if frameName.startswith('ar_marker_'):
		stamp = tfs.header.stamp.to_sec()
		print("stamp: {}".format(stamp))
		tagId = int(frameName.split('_')[2])
		trans = tfs.transform.translation
		trans = np.array([trans.x, trans.y, trans.z])
		rot = tfs.transform.rotation
		rot = np.array([rot.x, rot.y, rot.z, rot.w])
		tfRbt = self._return_rbt(trans, rot)
		rbt = self._tagRBTs[tagId].dot(tfRbt).dot(self._cam2fcuRBT) 
		print("got rbt for {}".format(frameName))
    #		update tag's latest timestamp and last known RBT
		#self._rbt_status_lock.acquire()
		print("cb lock")
		self._lastRbtUpdateTime[tagId] = stamp
		self._lastKnownRbt[tagId, :, :] = rbt
		#self._rbt_status_lock.release()

    def _return_rbt(self, trans, rot):
	omega, theta = eqf.quaternion_to_exp(rot)
	return eqf.create_rbt(omega, theta, trans)

if __name__ == '__main__':
    rospy.init_node('ar_cap', anonymous=False)
    poseEstimator = ARPoseEstimator(10, 10)
    poseEstimator.run()
