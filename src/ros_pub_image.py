#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

usb_cam = cv2.VideoCapture(0)
if not (usb_cam.isOpened()):
	print("Could not open video device")
# To set the resolution
usb_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
usb_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
pub = rospy.Publisher('camera/image', Image, queue_size=10)
rospy.init_node('image_topic_publisher_node', anonymous=True)
rate = rospy.Rate(10)  # 10hz

bridge = CvBridge()
while not rospy.is_shutdown():
	# Capture frame-by-frame
	ret, frame = usb_cam.read()
	# Display the resulting frame
	cv2.imshow('preview',frame)
	msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
	pub.publish(msg)
	rate.sleep()
	# Waits for a user input to quit the application
	if cv2.waitKey(1) & 0xFF == ord('q'):
		usb_cam.release()
		cv2.destroyAllWindows()
		break
