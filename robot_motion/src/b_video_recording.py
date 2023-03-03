#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(object):

    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/gazebo_camera/image_raw", Image, self.callback)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        ## Write your Opencv Image manipulation Code here

        ##Example
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        cv2.imshow("Gray Image", gray_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    subscriber = ImageSubscriber()
    rospy.spin()
