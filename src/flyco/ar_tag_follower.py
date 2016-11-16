import sys
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, Vector3
import std_msgs.msg
import rospy, tf
class ARTagFollower:
    def __init__(self, markerFrameList):
        self.markerFrameList = markerFrameList
        self.lastTrans = None
        self.lastRot = None
        self.lastCommandTimestamp = None
        self.hasFrame = False
        if len(markerFrameList) < 1:
            print("Error: input marker frame list")
            sys.exit()

        self.currentMarker = markerFrameList[0]
        self.publisher = rospy.Publisher("/setpoint_position/local", PoseStamped, queue_size = 1)
        self.tfListener = tf.TransformListener()

        self.rate = rospy.Rate(10.0)

    def send_follow_command(self):
        CAMERA_FRAME = "/usb_cam"
        markerFrame = "ar_marker_" + self.currentMarker

        try:
            latestTfTimestamp = self.tfListener.getLatestCommonTime(CAMERA_FRAME, markerFrame)
            (trans,rot) = self.tfListener.lookupTransform(markerFrame, CAMERA_FRAME, rospy.Time(0))
            if latestTfTimestamp != self.lastCommandTimestamp: # check if new tick
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                posePoint = Point(trans[0], trans[1], trans[2])
                poseRotation = Quaternion(rot[0], rot[1], rot[2], rot[3])
                pose = Pose(posePoint, poseRotation)
                cmd = PoseStamped(header,pose)
                self.publisher.publish(cmd)
                self.lastCommandTimestamp = latestTfTimestamp
                self.hasFrame = True
        except tf.Exception as e:
            if self.hasFrame == False:
                print("Frame {0} not found.".format(markerFrame))
            else:
                print(e)
            #TODO Impliment shutdown    
            
    
    def shutdown(self): # run shutdown safty sequence to avoid distruction of quad 
        sys.exit()
