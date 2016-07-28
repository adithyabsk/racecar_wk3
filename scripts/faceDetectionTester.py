#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
from racecar_wk3.msg import BlobDetections
from std_msgs.msg import String
from geometry_msgs.msg import Point
import math
import time


face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
im = cv2.imread("karaman.png")
gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
faces = face_cascade.detectMultiScale(gray, 1.3, 5)
found_face = False
for (x, y, w, h) in faces:
    roi_gray = gray[y:y+h, x:x+w]
    roi_color = im[y:y+h, x:x+w]
    eyes = eye_cascade.detectMultiScale(roi_gray)
    if len(eyes) == 0: continue
    ex, ey, ew, eh = eyes[0]
    cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(100,255,100),2)
    found_face = True
    cv2.rectangle(im, (x, y), (x+w, y+h), (100, 255, 100), 2)
    eh = eyes[0][3]
    print "face h: {},  eye h: {}".format(h, eh)
if found_face:
    cv2.imwrite("/home/racecar/challenge_photos/face{}.png".format(int(time.clock()*1000)), im)
    print "wrote face"
    rospy.sleep(1)
