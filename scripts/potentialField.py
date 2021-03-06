#!/usr/bin/env python

"""
potentialField.py

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


# CLASS DECLARATION

class potentialCommander:

    def __init__(self):
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        # Add any other topic variables here

        self.FORCE_CONSTANT = 0.1                           # equal to kQq in Coloumb's Law
        self.HOKUYO_ANGLES = np.arange(-45, 225.25, .25)      # creates a np array from -45 to 225 in 0.25 segments

        self.K_speed = 0.1
        self.K_angle = 1
        self.PUSH_VECTOR = 5
        
        
        # Add any other class variables here
    


    # Function: publish
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def publish(self, speed, angle):
        msg = AckermannDriveStamped()           # Initializes msg variable
        msg.drive.speed = speed                 # Sets msg speed to entered speed
        msg.drive.steering_angle = angle        # Sets msg steering angle to entered angle
        self.DrivePub.publish(msg)              # Publishes the message



    # Function: getX
    # Parameters: r, theta (floats)
    #
    # This function will convert a polar (r, theta)  
    # coordinate into a cartesian x coordinate

    def getX(self, r, theta):
        return r * math.cos(theta)


    # Function: getY
    # Parameters: r, theta (floats)
    #
    # This function will convert a polar (r, theta)  
    # coordinate into a cartesian y coordinate

    def getY(self, r, theta):
        return r * math.sin(theta)
        


    # Function: getR
    # Parameters: x, y (floats)
    #
    # This function will convert a cartesian (x, y)  
    # coordinate into a polar r coordinate

    def getR(self, x, y):
        return math.sqrt(x * x + y * y)
    

    
    # Function: getTheta
    # Parameters: x, y (floats)
    #
    # This function will convert a cartesian (x, y)  
    # coordinate into a polar theta coordinate

    def getTheta(self, x, y):
        return math.atan(y/x)
    


    # Function: drive
    # Parameters: msg (LaserScan)
    #
    # This function uses a PID control system
    # to let the car follow a line

    def drive(self, msg):
        
        ranges = np.array(msg.ranges)                           # turns msg ranges into np array
        magnitudes = self.FORCE_CONSTANT / np.power(ranges, 2)  # creates an array of magnitudes at each point

        xComponents = np.multiply(magnitudes, np.cos(np.deg2rad(self.HOKUYO_ANGLES)))   # creates array of x components
        yComponents = np.multiply(magnitudes, np.sin(np.deg2rad(self.HOKUYO_ANGLES)))   # creates array of y components

        netX = np.sum(xComponents)      # creates a net x scalar
        netY = np.sum(yComponents) + self.PUSH_VECTOR      # creates a net y scalar

        speed = self.K_speed * math.sqrt(netX * netX + netY * netY)*np.sign(netY)    
        direction = self.K_angle * math.atan2(netX, netY)*np.sign(netY)

        self.publish(speed, direction)
        

    

        """
        
        # Coulomb's Law:  F = (k * q1 * q2) / r^2
        #
        # For purposes of this algorithm, kq1q2 is a constant, K
        # and r is the distance of each point

        rValues = []

        xValues = []
        yValues = []

        index = 0

        finalX = 0
        finalY = 0
        finalTheta = 0
        finalR = 0

        K = 2

        # Convert msg ranges into proportional r values
        for item in msg.ranges:
            rValues.append(K/(item * item))

        # CRITICAL ASSUMPTION
        #
        # Counting begins at 0 degrees
        # LET 0 DEGREES = -45 IN THE LASERSCAN DATA
        
        for item in rValues:

            theta = index / 4
            
            if theta > 135:
                xValues.append(self.getX(item * -1, theta))  # -x

            else:
                xValues.append(self.getX(item, theta))
                

            if ((theta < 45) or (theta > 225)):
                yValues.append(self.getY(item * -1, theta))  # -y

            else:
                yValues.append(self.getY(item, theta))
            
            index += 1


        # Add gigantic forwards x vector
        xValues.append(1)
        yValues.append(0)
        
            
        # Calculate resultant x vector
        for item in xValues:
            finalX += item

        finalX / len(xValues)


        # Calculate resultant y vector
        for item in yValues:
            finalY += item

        finalY / len(yValues)
            

        # Calculate resultant drive vector
        if finalX < 0:
            driveAngle = -(math.atan(finalY / abs(finalX)))
            driveSpeed = (math.sqrt(finalX * finalX + finalY * finalY))/4

        else:
            driveAngle = (math.atan(finalY / abs(finalX)))
            driveSpeed = (math.sqrt(finalX * finalX + finalY * finalY))/4

        self.publish(driveSpeed, driveAngle)
        """
        





