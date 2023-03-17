#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
from xihelm_task.srv import MotorControl

class MotorControlNode:
    def __init__(self):
        # initialize the ROS node
        rospy.init_node('motor_control_node', anonymous=True)

        # create a ROS service to start/stop the motors
        self.motor_service = rospy.Service('motor_control', MotorControl, self.handle_motor_control)

        # create a subscriber to listen for error messages from the object detection nodes
        rospy.Subscriber('object_detection_error', String, self.handle_error)

        # initialize the motor state to "running"
        self.motor_running = True

    def handle_motor_control(self, req):
        # handle the start/stop motor request
        self.motor_running = req.enable
        return SetBoolResponse(success=True, message='Motor control set to {}'.format('on' if self.motor_running else 'off'))

    def handle_error(self, msg):
        # handle the error message from the object detection nodes
        if self.motor_running and 'bad object detected' in msg.data:
            # stop all motors if they are currently running
            rospy.loginfo('Stopping all motors')
            # code to stop motors goes here

if __name__ == '__main__':
    try:
        node = MotorControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
