# Conveyor_Camera_ROS
[Watch the video](https://youtu.be/IpvomwK4SoU)

This is overview of a possible architecture for a system that can detect "bad objects" and stop all motor functions using multiple cameras, ROS, and Python:
## 1. Camera Nodes: 
Each camera will be simulated as a ROS node. These nodes will capture images and publish them on a ROS topic. To simulate the detection of "bad objects," the camera nodes will randomly or time-basedly publish an error message on a separate ROS topic.
## 2.Object Detection Nodes: 
These nodes will receive the images published by the camera nodes, and using computer vision techniques, they will analyze the images to detect any "bad objects." If any "bad objects" are detected, the nodes will publish a message on a separate ROS topic to trigger the motor control node to stop all motors.
## 3.Motor Control Node:
This node will receive the error messages published by the object detection nodes and stop all motor functions when it receives the error message. The motor control node will implement a ROS service that can be called to start or stop the motors.
## 4.Main Controller Node: This node will be responsible for connecting the camera nodes, object detection nodes, and motor control node. It will subscribe to the error messages published by the object detection nodes and call the motor control node's service to stop all motor functions if any "bad objects" are detected.

With this architecture, we can have a scalable and modular system that can be extended to accommodate more cameras, and it can also handle different types of "bad objects" by changing the object detection nodes.
However, the system might break if the cameras fail or the network connection between the nodes is lost. Therefore, we need to implement error handling mechanisms to make the system more robust.

## Explanation – 
### camera_node.py
This is an example implementation of a ROS node called CameraNode, which simulates a camera that publishes image data and error messages.
The node uses the cv2.VideoCapture() method to capture video frames from the default camera (ID 0) of the computer. Then, it converts each captured frame to a ROS Image message and publishes it to the topic /camera/image_raw.
In addition to publishing the image data, the node also publishes error messages to the topic /camera/error with a probability of 0.1 using the rospy.Publisher() method.
The run() method of the CameraNode class contains a loop that runs until the rospy.is_shutdown() condition is met. Within the loop, it reads a frame from the camera and publishes it along with an error message if applicable. The rospy.sleep(0.05) statement at the end of the loop causes the node to sleep for 50 milliseconds before repeating the loop.
Finally, in the if __name__ == '__main__': block, the node is initialized with rospy.init_node('camera_node'), and an instance of the CameraNode class is created and run with node.run().
### object_detection_node.py
The script starts by importing necessary ROS and OpenCV libraries - rospy and cv2. It also imports some message types (Image and String) and the CvBridge class from the cv_bridge package which is used to convert between ROS messages and OpenCV images.
The ObjectDetector class is defined which initializes the ROS node with a given name ('object_detector') and sets up two ROS topics:
/camera/image - a subscriber to receive images from the camera
/error - a publisher to publish any error messages related to object detection
The image_callback function is called each time a new image is received on the /camera/image topic. The function first converts the received ROS image message to a OpenCV image format using the CvBridge class. The image is then processed using computer vision techniques to detect any objects of interest. If a "bad object" is detected, the node publishes an error message on the /error topic using the String message type.
Finally, the script initializes the ObjectDetector class and enters the ROS event loop (rospy.spin()) to process incoming messages and publish messages on the two topics. The try-except block is used to catch any ROSInterruptException exceptions that may occur during execution of the script.

### motor_control_node.py
This is a Python script for a ROS node that controls motors based on input from other nodes in the ROS network. The node subscribes to a topic called "object_detection_error" to listen for error messages from other nodes, and if it receives a message indicating a "bad object detected" error and the motors are currently running, it will stop all motors.
The node also provides a ROS service called "motor_control" that allows other nodes to start or stop the motors. When the service is called, the node updates the state of the motor (running or stopped) and returns a response indicating whether the operation was successful.
Overall, this node provides a centralized motor control service that can be accessed by other nodes in the ROS network. By subscribing to the error messages from other nodes, it can detect when to stop the motors based on the current state of the system.

### main_controller_node.py
This is a ROS node written in Python that subscribes to a ROS topic 'object_detection_error' and calls a ROS service 'motor_control' when a 'bad object detected' error message is received. The node waits for the 'motor_control' service to be available before subscribing to the topic. The handle_error method of the node is responsible for stopping all motors by calling the 'motor_control' service with a False argument. The response message from the service is logged. If an exception occurs while calling the service, an error message is logged. The node is initialized as 'main_controller_node'.
