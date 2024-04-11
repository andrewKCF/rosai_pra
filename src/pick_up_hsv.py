#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self.cv_image = None
        cv2.namedWindow("HSV value")
        cv2.setMouseCallback("HSV value", self.on_mouse)
    
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("HSV value", self.cv_image)
            cv2.waitKey(1)
        except Exception as e:
            print(e)
    
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
        #if event == cv2.EVENT_MOUSEMOVE:
            if self.cv_image is not None:
                hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
                hsv_value = hsv[y, x]
                print("HSV value at point (x={}, y={}): {}".format(x, y, hsv_value))
                # show hsv value
                #cv2.putText(self.cv_image, "HSV value: {}".format(hsv_value), (10, self.cv_image.shape[0] - 10),
                            #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                #cv2.imshow("HSV value", self.cv_image)
                colorLow = hsv_value-10
                colorHigh = hsv_value+10
                mask = cv2.inRange(hsv, colorLow, colorHigh)
                cv2.imshow("mask", mask)

if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
