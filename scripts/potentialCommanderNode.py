#!/usr/bin/env python



# IMPORTS

import rospy
import math

from potentialCommander import potentialCommander

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#from racecar_wk3.msg import BlobDetections
#from racecar_wk3.msg import ObjectDetections



# VARIABLES

LASER_TOPIC = "/racecar/laser/scan"
#OBJECT_TOPIC = "/objects"
#BLOB_TOPIC = "/blobs"
#MAP_TOPIC = "/map"

NODE_NAME = 'potentialCommander'

#D_DESIRED = 0.8

cody = potentialCommander()


# CALLBACK

def callBack(laser_msg):

    cody.drive(laser_msg)


# MAIN()      

rospy.init_node(NODE_NAME)

laser_msg = rospy.Subscriber(LASER_TOPIC, LaserScan, callBack)
#object_msg = rospy.Subscriber(OBJECT_TOPIC, LaserScan, callBack)
#blob_msg = rospy.Subscriber(BLOB_TOPIC, BlobDetections, callBack)
#map_msg = rospy.Subscriber(MAP_TOPIC, MapDetections, callBack)

#sideEntry = rospy.Subscriber("", ,)

rospy.spin()
