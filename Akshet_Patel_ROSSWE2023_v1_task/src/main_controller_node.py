#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from xihelm_task.srv import MotorControl

class MainControllerNode:
    def __init__(self):
        # initialize the ROS node
        rospy.init_node('main_controller_node', anonymous=True)

        # create a subscriber to listen for error messages from the object detection nodes
        rospy.Subscriber('object_detection_error', String, self.handle_error)

        # create a ROS service proxy to start/stop the motors
        rospy.wait_for_service('motor_control')
        self.motor_service = rospy.ServiceProxy('motor_control', MotorControl)

    def handle_error(self, msg):
        # handle the error message from the object detection nodes
        if 'bad object detected' in msg.data:
            # stop all motors if they are currently running
            rospy.loginfo('Stopping all motors')
            try:
                response = self.motor_service(False)
                rospy.loginfo(response.message)
            except rospy.ServiceException as e:
                rospy.logerr('Failed to call motor control service: {}'.format(str(e)))

if __name__ == '__main__':
    try:
        node = MainControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
