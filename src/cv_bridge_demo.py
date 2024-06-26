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
        rospy.init_node("cv_bridge_demo")
        # 创建cv_bridge对象
        self.bridge = CvBridge()
        # 订阅图像话题
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # 创建图像话题的发布端，用来发布将OpenCV图像转化成sensor_msgs/Image的图像
        self.image_pub = rospy.Publisher("/face_img",Image, queue_size=10)

        #read smiel file
        r=rospkg.RosPack()
        r_path=r.get_path('rosai_pra')+'/src/'+'smil_face.png'
        self.smile= cv2.imread(r_path)
        self.sh,self.sw=self.smile.shape[:2]

        rospy.loginfo("cv_bridge_demo Python demo is ready ......")

    def image_callback(self, data):
        # 使用cv_bridge()将ROS图像转换成OpenCV格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

        # 打开一个窗口显示图像，
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # 将画上长方形的图像转换回ROS图像消息格式，并发布到/cv_bridge_demo/image_show话题
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_demo node.")
        cv2.DestroyAllWindows()
