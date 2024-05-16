#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, cv2
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Load the cascade
h=rospkg.RosPack()
h_path=h.get_path('rosai_pra')+'/src/'+'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(h_path)

class cvBridgeDemo():
    def __init__(self):
        rospy.init_node("face_detection")
        # create cv_bridge
        self.bridge = CvBridge()
        # subsribe
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # publish
        self.image_pub = rospy.Publisher("/face_img",Image, queue_size=10)

        #read smile face
        r_path='/home/ettc/Desktop/face.jpeg'
        self.smile= cv2.imread(r_path)
        self.sh,self.sw=self.smile.shape[:2]

        rospy.loginfo("face detction is ready ......")

    def image_callback(self, data):
        # cv_bridge() ros image to opencv image
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)


        cv2.imshow("Image window", img)
        cv2.waitKey(1)
        #try:
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(face_img, "bgr8"))
        #except CvBridgeError as e:
            #print(e)

if __name__ == '__main__':
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down face_detection node.")
        cv2.DestroyAllWindows()
