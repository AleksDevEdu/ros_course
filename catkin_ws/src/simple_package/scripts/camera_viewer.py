#!/usr/bin/env python
# license removed for brevity
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(ros_data):
    # print('received image of type: "%s"' % ros_data.format)

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow('cv_img', cv_image)
    cv2.waitKey(3)


def listener():
    rospy.init_node('camera_viewer', anonymous=True)
    subscriber = rospy.Subscriber('camera_image_raw', Image, callback,  queue_size = 1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exception caught')