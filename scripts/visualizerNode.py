#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from racecar_wk3.msg import BlobDetections
from racecar_wk3.msg import ObjectDetections

class visualizerNode:
    def __init__(self):
        rospy.set_param('~', False)
        rospy.set_param('~', False)
        rospy.set_param('~', False)
        rospy.set_param('~', False)
        rospy.set_param('~', False)
        rospy.set_param('~', False)


        private_param = rospy.get_param('~private_name')

