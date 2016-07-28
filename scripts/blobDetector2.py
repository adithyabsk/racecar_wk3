#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
from red_light_green_light.msg import BlobDetections
from std_msgs.msg import ColorRGBA, Float64
from geometry_msgs.msg import Point
import math


class BlobDetector:
    def __init__(self):
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("blobs", BlobDetections, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("BlobDetector initialized.")

    def cbImage(self, image_msg):
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False): return
        im = self.bridge.imgmsg_to_cv2(image_msg)
        im = im[len(im)*.4:]
	hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        self.msg = BlobDetections()
        self.find_color(im, "red", cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 110, 180]), np.array([15, 255, 255])), cv2.inRange(hsv, np.array([175, 110, 180]), np.array([180, 255, 255]))))  # red
        self.find_color(im, "green", cv2.inRange(hsv, np.array([40, 55, 140]), np.array([85, 185, 250])))  # green
        self.find_color(im, "yellow", cv2.inRange(hsv, np.array([20, 55, 140]), np.array(35, 185, 250])))  # yellow
        self.find_color(im, "blue", cv2.inRange(hsv, np.array([110, 55, 140]), np.array(130, 185, 250])))  # blue
        self.pub_image.publish(self.msg)
        self.thread_lock.release()

    def find_color(self, im, label_color, mask):
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        approx_contours = []
        for c in contours:
	    area = cv2.contourArea(c)
            if area < 100: continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .05*perim, True)
            if len(approx) == 4:
                approx_contours.append(approx)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                cv2.circle(im, center, 3, (255, 100, 100), 4)
                print "Moment:  ({}, {})".format(center[0], center[1])

                self.msg.colors.append(label_color)
                print "Label color:  {}".format(label_color)
                self.msg.heights.append(float((max(approx, key=lambda x: x[0][1])[0][1] - min(approx, key=lambda x: x[0][1])[0][1])) / len(im[0]))
                print "Height:  {}".format(self.msg.heights[-1])
                msg_loc = Point()
                msg_loc.x, msg_loc.y = float(center[0]) / len(im[0]), float(center[1]) / len(im)
                self.msg.locations.append(msg_loc)


if __name__ == "__main__":
    rospy.init_node("BlobDetector")
    bd = BlobDetector()
    rospy.spin()
