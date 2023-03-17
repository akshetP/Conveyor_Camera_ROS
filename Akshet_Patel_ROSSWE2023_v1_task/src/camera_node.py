#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np

class CameraNode:
    def __init__(self):
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.error_pub = rospy.Publisher('/camera/error', String, queue_size=10)
        self.cap = cv2.VideoCapture(0)

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                # Convert the frame to a ROS image message and publish it
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'camera'
                msg.height, msg.width, _ = frame.shape
                msg.encoding = 'bgr8'
                msg.step = 3 * msg.width
                msg.data = np.array(frame).tostring()
                self.image_pub.publish(msg)

                # Randomly publish an error message
                if np.random.rand() < 0.1:
                    self.error_pub.publish('Error: Bad object detected')
            
            rospy.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node('camera_node')
    node = CameraNode()
    node.run()
