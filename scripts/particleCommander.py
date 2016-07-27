#!/usr/bin/env python
"""
particleCommander(git).py

MIT RACECAR 2016

This class will act as a commanding node that reads
(processed) message data from accompanying nodes, makes
decisions on how to proceed, then publishes drive commands
directly to the /navigation topic.

"""


# IMPORTS

import rospy
import math
import time
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#from racecar_wk3.msg import BlobDetections
#from racecar_wk3.msg import ObjectDetections


# CLASS DECLARATION

class ParticleCommander:

    def __init__(self):
        #self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        self.DrivePub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10) #gazebo pulisher
        # Add any other topic variales here

        #self.SPEED = 0.5

        # Add any other class constants here

        self.PUSH_VECTOR = np.array([0, 100])
        self.FORCE_CONSTANT = 20 # the numerator in the F = (kQq)/r^2
        self.HOKUYO_ANGLES = np.arange(-45,225,0.25) #change back to np.arange(-45,225.25,0.25)
        #print len(self.HOKUYO_ANGLES)
        # Add any other class variables here

    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, direction, magnitude):
        msg = AckermannDriveStamped()           # Initializes msg variable
        msg.drive.speed = magnitude             # Sets msg speed to entered speed
        #msg.drive.acceleration = 1              # Sets msg acceleration to 0
        #msg.drive.jerk = 1                      # Sets msg jerk to 1
        msg.drive.steering_angle = direction    # Sets msg steering angle to entered angle
        #msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
        self.DrivePub.publish(msg)              # Publishes the message
        
    def avoidObjects(self, msg):
        #parameters speed should go from -1 to 1
        #Calculate Force Vector
        ranges = np.array(msg.ranges) #Only range values at this point
        squared = np.power(ranges, 2)
        forces = self.FORCE_CONSTANT / squared
        xComp = np.multiply(np.copy(forces), np.cos(np.deg2rad(self.HOKUYO_ANGLES)))
        yComp = np.multiply(np.copy(forces), np.sin(np.deg2rad(self.HOKUYO_ANGLES)))
        fVec = np.column_stack((xComp, yComp))
        finalFVec = np.negative(np.sum(fVec, axis=0))

        #Calculate Resultant Force
        rsltFVec = np.add(finalFVec, self.PUSH_VECTOR)

        #Convert rectangular to polar
        direction = np.arctan(rsltFVec[1]/rsltFVec[0])
        print direction
        magnitude = np.sqrt(rsltFVec[0]**2+rsltFVec[1]**2)
        print magnitude
        
        self.drive(direction, magnitude)

    
