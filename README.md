# rosai_pra

camera-capture /dev/video0 --input-width=640 --input-height=360

roslaunch ros_deep_learning video_viewer.ros1.launch input:=/dev/video0

roslaunch ros_deep_learning imagenet.ros1.launch input:=/dev/video0

roslaunch ros_deep_learning detectnet.ros1.launch input:=/dev/video0

catkin_make --pkg rosai_pra

catkin_create_pkg rosai_pra std_msgs rospy roscpp sensor_msgs cv_bridge

catkin_create_pkg rosai_pra std_msgs rospy roscpp sensor_msgs


jetettc

roslaunch usb_cam usb_cam-test.launch

rosrun rosai_pra color_detection.py

rosrun rqt_reconfigure rqt_reconfigure
