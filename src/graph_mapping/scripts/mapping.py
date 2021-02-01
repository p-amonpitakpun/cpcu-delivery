#!/usr/bin/env python

import cv2
import message_filters
import rospy

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge



bridge = CvBridge()
img = None

def callback(image, laser_scanner):
    print(image)

def image_callback(data):
    global img
    try:
        image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        img = image
    except e:
        print(e)

def main():
    cv2.namedWindow('image')
    # rospy.init_node('mapping', anonymous=True)

    # image_sub = message_filters.Subscriber('/image', Image)
    # laser_scanner_sub = message_filters.Subscriber('laser_scanner', Float32MultiArray)

    # time_sync = message_filters.TimeSynchronizer([image_sub, laser_scanner_sub], 1)
    # time_sync.registerCallback(callback)

    rospy.init_node('immage', anonymous=True)

    rospy.Subscriber('/image', Image, image_callback)

    while not rospy.is_shutdown():
        if img is not None:
            cv2.imshow('image', img)
            cv2.waitKey(3)

    rospy.spin()


if __name__ == "__main__":
    print('!')
    main()
