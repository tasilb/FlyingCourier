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

    def __init__(self, h, w, timeWindowSize = .2, timeout = 10.0, frequency = 10.0):
	self._ar_marker_size_in_m = 19.5 / 100.0
        self._ar_marker_spacing_in_m = .5 * 2.54 / 100.0 # half inch to meters
	self._zero_rotation = np.array([0, 0, 0, 1])
	self._numTags = h * w
	self._coord2Tag = np.arange(self._numTags).reshape((h, w))
	self._lastKnownRbt = np.zeros((self._numTags, 4, 4))
	self._lastRbtUpdateTime = np.zeros(self._numTags)
	self._rbt_status_lock = Lock()
        self.timeWindowSize = timeWindowSize
        self.timeout = timeout
        self.frequency = frequency

	# cache RBT from usb_cam to fcu
	self._tfListener = tf.TransformListener()
	self._tfListener.waitForTransform('fcu', 'usb_cam', rospy.Time(), rospy.Duration(self.timeout))
	camTofcuTrans, camTofcuRot = self._tfListener.lookupTransform('fcu', 'usb_cam', rospy.Time())
	self._camTofcuRBT = self._return_rbt(camTofcuTrans, camTofcuRot)

	# cache grid RBTs as matrix exponentials
	self._tagToOriginRBTs = np.zeros((self._numTags, 4, 4))
	print("Building RBT array for {} markers ({}, {})".format(self._numTags, h, w))
	for i in np.arange(h):
	    for j in np.arange(w):
		transX = (self._ar_marker_size_in_m + self._ar_marker_spacing_in_m) * j 
		transY = -(self._ar_marker_size_in_m + self._ar_marker_spacing_in_m) * i
		trans = np.array([transX, transY, 0.0])
		self._tagToOriginRBTs[self._coord2Tag[i, j], :, :] = self._return_rbt(trans, self._zero_rotation)


	self._tf_sub = rospy.Subscriber('/tf', tfMessage, self._on_tf, queue_size=1)
	self._mocap_pub = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=1)

    # estimation cycle
    def run(self):
	estimationRate = rospy.Rate(self.frequency)
	while not rospy.is_shutdown():
	    self._estimate_pose()
	    estimationRate.sleep()


    def _estimate_pose(self):
	estimatedPose = PoseStamped()
	estimatedPose.header.stamp = rospy.Time.now()
	estimatedPose.header.frame_id = 'ar_grid'
	# lock and copy, then unlock
	# self._rbt_status_lock.acquire()
	lastRbtUpdateTimeSnapshot = np.copy(self._lastRbtUpdateTime)
	lastKnownRbtSnapshot = np.copy(self._lastKnownRbt)
	# self._rbt_status_lock.release()

	# filter indices by timestamp
	minStamp = self._tfListener.getLatestCommonTime('fcu', 'usb_cam').to_sec() - self.timeWindowSize
	indicesInWindow = np.where(lastRbtUpdateTimeSnapshot >= minStamp)[0]

	# drop outliers and average
	# skipping outlier cleaning for now
	# normalize RBTs?
	rbtsInWindow = lastKnownRbtSnapshot[indicesInWindow]
	
        # assuming all rotations are same, relies on dropout
	if len(rbtsInWindow) > 0:
	    print("Filtering over {} transforms".format(len(indicesInWindow)))
	    avgTrans = np.average(rbtsInWindow, axis=0)[:, 3][:3]
            avgQuat = np.zeros(4)
            for rbt in rbtsInWindow:
                rot = tf.transformations.rotation_from_matrix(rbt)
                quat = tf.transformations.quaternion_about_axis(rot[0],rot[1])
                avgQuat += quat
	    avgQuat /= float(len(rbtsInWindow)) 

            # publish pose based on origin and regularized RBT
            estimatedPose.pose.position.x = avgTrans[0]
            estimatedPose.pose.position.y = avgTrans[1]
            estimatedPose.pose.position.z = avgTrans[2]
	    estimatedPose.pose.orientation.x = avgQuat[0]
	    estimatedPose.pose.orientation.y = avgQuat[1]
	    estimatedPose.pose.orientation.z = avgQuat[2]
	    estimatedPose.pose.orientation.w = avgQuat[3]
            print("TRANS", avgTrans[0], avgTrans[1], avgTrans[2])
            print("ROT", np.array(tf.transformations.euler_from_quaternion(avgQuat)) * 180 / np.pi)
    
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
		tagId = int(frameName.split('_')[2])
		trans = tfs.transform.translation
		trans = np.array([trans.x, trans.y, trans.z])
		rot = tfs.transform.rotation
		rot = np.array([rot.x, rot.y, rot.z, rot.w])
		tfRbt = np.linalg.inv(self._return_rbt(trans, rot))
		rbt = self._tagToOriginRBTs[tagId].dot(tfRbt).dot(self._camTofcuRBT) 

   		# update tag's latest timestamp and last known RBT
		# self._rbt_status_lock.acquire()
		self._lastRbtUpdateTime[tagId] = stamp
		self._lastKnownRbt[tagId, :, :] = rbt
		# self._rbt_status_lock.release()

    def _return_rbt(self, trans, rot):
	omega, theta = eqf.quaternion_to_exp(rot)
	return eqf.create_rbt(omega, theta, trans)

if __name__ == '__main__':
    rospy.init_node('ar_cap', anonymous=False)
    poseEstimator = ARPoseEstimator(10, 10)
    poseEstimator.run()
