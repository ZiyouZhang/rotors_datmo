"""
/**
 * @file apriltag_listener.py
 * @author Ziyou Zhang (ziyou.zhang@outlook.com)
 * @brief Python implementation for apriltag detection.
 * @version 0.1
 * @date 2020-09-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
"""

#!/usr/bin/env python
import numpy as np
import apriltag
import rospy
import roslib
import cv2
import message_filters
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

VERBOSE = True

def callback(data):
    # print(data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    detector = apriltag.Detector()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "mono8")
    # image = cv2.imread(data, cv2.IMREAD_GRAYSCALE)
    # print(image)
    detections = detector.detect(image)
    print(detections[0])

def detect():
    detector = apriltag.Detector()

    imagepath = "/home/ziyou/Desktop/test1.png"
    image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    detections = detector.detect(image)
    print(detections)

    imagepath = "/home/ziyou/Desktop/test2.png"
    image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    detections = detector.detect(image)
    print(detections)

    imagepath = "/home/ziyou/Desktop/test3.png"
    image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    detections = detector.detect(image)
    print(detections)


def detector():
    rospy.init_node('tag_detector', anonymous=True)
    rospy.Subscriber("/depth_camera/image_raw", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    detector()
