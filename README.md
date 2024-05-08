# rosai_pra 
cd ~/ros_inference/src

sudo rm -rf ~/ros_inference/src/rosai_pra

git clone https://github.com/andrewKCF/rosai_pra.git

roslaunch rosai_pra usb_cam.launch

rosrun rosai_pra pick_up_bgr.py

rosrun rosai_pra pick_up_hsv.py

rosrun rosai_pra color_detection_hsv.py

rosrun rosai_pra color_detection_bgr.py 

camera-capture /dev/video0 --input-width=640 --input-height=360

roslaunch ros_deep_learning video_viewer.ros1.launch input:=/dev/video0

roslaunch ros_deep_learning imagenet.ros1.launch input:=/dev/video0

roslaunch ros_deep_learning detectnet.ros1.launch input:=/dev/video0

catkin_make --pkg rosai_pra

catkin_create_pkg rosai_pra std_msgs rospy roscpp sensor_msgs cv_bridge

catkin_create_pkg rosai_pra std_msgs rospy roscpp sensor_msgs

jetettc


