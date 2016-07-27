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

<<<<<<< HEAD
from commander import ParticleCommander
=======
from commander import Commander
>>>>>>> c5376eaed2002d0f88e8f4fbbbaf4096089c29eb

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#from racecar_wk3.msg import BlobDetections
#from racecar_wk3.msg import ObjectDetections



# VARIABLES

#OBJECT_TOPIC = "/scan"
OBJECT_TOPIC = "/racecar/laser/scan"
#BLOB_TOPIC = "/blobs"
#MAP_TOPIC = "/map"

NODE_NAME = 'commander'

#D_DESIRED = 0.8

commander = ParticleCommander()

def callback(msg):
    cody.evade_objects(msg)

# MAIN()      

rospy.init_node(NODE_NAME)

<<<<<<< HEAD
object_sub = rospy.Subscriber(OBJECT_TOPIC, LaserScan, commander.avoidObjects)
=======
object_sub = rospy.Subscriber(OBJECT_TOPIC, ObjectDetections, callback)
>>>>>>> c5376eaed2002d0f88e8f4fbbbaf4096089c29eb
#blob_msg = rospy.Subscriber(BLOB_TOPIC, BlobDetections, callBack)
#map_msg = rospy.Subscriber(MAP_TOPIC, MapDetections, callBack)

#sideEntry = rospy.Subscriber("", ,)

rospy.spin()
