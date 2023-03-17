#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.error_pub = rospy.Publisher('/error', String, queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # perform object detection using computer vision techniques
        if bad_object_detected:
            self.error_pub.publish('Error: Bad object detected')

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
