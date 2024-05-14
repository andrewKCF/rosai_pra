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
        # 创建cv_bridge对象
        self.bridge = CvBridge()
        # 订阅图像话题
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # 创建图像话题的发布端，用来发布将OpenCV图像转化成sensor_msgs/Image的图像
        self.image_pub = rospy.Publisher("/face_img",Image, queue_size=10)

        #read smile face
        r=rospkg.RosPack()
        r_path=r.get_path('rosai_pra')+'/src/'+'smil_face.png'
        print (r_path)

        self.smile= cv2.imread(r_path)
        self.sh,self.sw=self.smile.shape[:2]

        rospy.loginfo("face detction is ready ......")

    def image_callback(self, data):
        # 使用cv_bridge()将ROS图像转换成OpenCV格式
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        # Draw rectangle around the faces
        face_img=self.smile
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            scale_w=float(w)/self.sw
            scale_h=float(h)/self.sh
            smile_rz=cv2.resize(self.smile,None,fx=scale_w,fy=scale_h)
            face_img=img[y:y+h,x:x+w].copy()
            roi=img[y:y+h,x:x+w]
            face=roi
            roi[:]=0
            roi[:]=cv2.add(roi,smile_rz)
            cv2.imshow("face", face_img)

        cv2.imshow("Image window", img)

        cv2.waitKey(1)


        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(face_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down face_detection node.")
        cv2.DestroyAllWindows()
