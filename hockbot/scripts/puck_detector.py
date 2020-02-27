#!/usr/bin/env python
#
#   puck_detector
#
#   Run a Cascade Object Detector in OpenCV.
#
#   Subscribers:    /image              Source image, to be remapped
#   Publishers:     /detector/image     Destination image
#                   /detector/???       Coordinates??
#
#   Services:       none
#

# ROS Imports
from computer_vision import process_image
import rospy
import sensor_msgs.msg
from geometry_msgs.msg import PointStamped
import cv2
import cv_bridge
import numpy as np

#
#  Detector Node Class
#
class Detector:
    def __init__(self):

        # Set up the OpenCV Bridge.
        self.bridge = cv_bridge.CvBridge()

        # Pick the topic names.  The source image topic can be
        # remapped in the command line.  The '~' places the output
        # image topic will be under the node name.
        source_topic = rospy.resolve_name("image")
        output_topic = rospy.resolve_name("~image")
        point_output_topic = rospy.resolve_name("/detector/strikers")

        # Subscribe to the source topic.  Using a queue size of one
        # means only the most recent message is stored for the next
        # subscriber callback.
        rospy.Subscriber('/cv_camera/image_raw',
                         sensor_msgs.msg.Image,
                         self.process,
                         queue_size=1)

        # Publish to the output topic.
        self.publisher = rospy.Publisher(output_topic,
                                         sensor_msgs.msg.Image,
                                         queue_size=1)

        self.point_publisher = rospy.Publisher(point_output_topic,
                                                PointStamped,
                                                queue_size=1)

        # Report.
        rospy.loginfo("Detector configured with:")
        rospy.loginfo("Image source topic: " + source_topic)
        rospy.loginfo("Image output topic: " + output_topic)

    def process(self, rosImage):
        # Convert into OpenCV image.
        cvImage = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")

        # Convert to gray scale.
        gray = cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)

        # Run the detector.
        objects = process_image(gray)

        # Convert back into a ROS image and republish (for debugging).
        self.publisher.publish(
            self.bridge.cv2_to_imgmsg(objects, "bgr8"))


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.  You can override the name using the
    # 'rosrun .... __name:=something' convention.
    rospy.init_node('detector')

    # Instantiate the Detector object.
    detector = Detector()

    # Continually process until shutdown.
    rospy.loginfo("Continually processing latest pending images...")
    rospy.spin()

    # Report completion.
    rospy.loginfo("Done!")