#!/usr/bin/env python


"""
commanderNode.py

MIT RACECAR 2016

This program implements the commander class and manages
the advanced computations required by the RACECAR

"""

# IMPORTS

import rospy
import math
from commander import commander
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



# VARIABLES

#OBJECT_TOPIC = "/objects"
#BLOB_TOPIC = "/blobs"
#MAP_TOPIC = "/map"

NODE_NAME = 'commander'

D_DESIRED = 0.8

cody = commander()


# CALLBACK

def callBack(msg):

    # Query for P Wall Follow Controller
    #kevin.PWallFollow(msg.ranges, D_DESIRED, SPEED, side)


# MAIN()      

rospy.init_node(NODE_NAME)
#scanResult = rospy.Subscriber(DATA_THREAD, LaserScan, callBack)

#sideEntry = rospy.Subscriber("", ,)

rospy.spin()
