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

from ackermann_msgs.msg import AckermannDriveStamped

# CLASS DECLARATION

class Commander:

    def __init__(self):
        #self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        self.DrivePub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10) # The Gazebo Pulisher

        # CLASS CONSTANTS
        self.PUSH_VECTOR = np.array([0, 1])                                                # The push vector (negative charge) behind the car
        self.FORCE_CONSTANT = 0.1                                                          # The FORCE_CONSTANT is equal to kQq in the equation F = (kQq)/r^2
        self.P_DIR = 1.0                                                                   # The P Gain for the direction control of the car
        self.P_MAG = 0.03                                                                  # The P Gain for the angle control of the car
        self.HOKUYO_ANGLES = np.arange(-45,225,0.25)                                       # Generates 1080 samples
    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, direction, magnitude):
        msg = AckermannDriveStamped()                                                      # Initializes msg variable
        msg.drive.speed = magnitude                                                        # Sets msg speed to entered speed
        msg.drive.steering_angle = direction                                               # Sets msg steering angle to entered angle
        self.DrivePub.publish(msg)                                                         # Publishes the message

    # Function: avoidObjects
    # Parameters: msg (LaserScan)
    #
    # This function is the callback function for the LaserScan
    # message passed to it.

    def avoidObjects(self, msg):
        ranges = np.array(msg.ranges)                                                      # Convert the ranges to a numpy array
        forces = self.FORCE_CONSTANT / np.power(ranges, 2)                                 # Calculate the "psedo-force" of every object on the racecar
        xComp = np.multiply(forces, np.cos(np.deg2rad(self.HOKUYO_ANGLES)))                # Caculate the x components of the force vector on the car
        yComp = np.multiply(forces, np.sin(np.deg2rad(self.HOKUYO_ANGLES)))                # Caculate the y components of the force vector on the car
        finalFVec = np.negative([np.sum(xComp), np.sum(yComp)])                            # Calculate the final vector on the car and negate it

        rsltFVec = np.add(finalFVec, self.PUSH_VECTOR)                                     # Calculate Resultant Force

        direction = self.P_DIR*math.atan2(rsltFVec[1],rsltFVec[0])                         # Calculate the direction of the car (angle)
        magnitude = self.P_MAG*np.sqrt(rsltFVec[0]**2+rsltFVec[1]**2)*np.sign(rsltFVec[0]) # Calculate the magnitude of the car (speed)
        
        self.drive(direction, magnitude)                                                   # Give the car the Drive command