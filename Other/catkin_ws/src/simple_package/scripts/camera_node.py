#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('camera_node', anonymous=True)

videoPub = rospy.Publisher('camera_image_raw', Image, queue_size=10)
rate = rospy.Rate(30) # 1hz

cam = cv2.VideoCapture(0)
cam.set(3,320);
cam.set(4,240);

bridge = CvBridge()

def talker():
    while cam.isOpened() and not rospy.is_shutdown():
        meta, frame = cam.read()

        frame_gaus = cv2.GaussianBlur(frame, (3, 3), 0)

        # frame_gray = cv2.cvtColor(frame_gaus, cv2.COLOR_BGR2GRAY)

        # I want to publish the Canny Edge Image and the original Image
        videoPub.publish(bridge.cv2_to_imgmsg(frame_gaus, "bgr8"))

        rate.sleep()
        # time.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exception caught')