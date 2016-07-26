#!/usr/bin/env python
"""
commander(git).py

MIT RACECAR 2016

™™This class will act as a commanding node that reads
(processed) message data from accompanying nodes, makes
decisions on how to proceed, then publishes drive commands
directly to the /navigation topic.

"""


# IMPORTS

import rospy
import math
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from racecar_wk3.msg import BlobDetections
from racecar_wk3.msg import ObjectDetections


# CLASS DECLARATION

class commander:

    def __init__(self):
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        # Add any other topic variables here

        self.SPEED = 3.0
        # Add any other class constants here

        self.prev_error = 0
        # Add any other class variables here
    


    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, angle):
        msg = AckermannDriveStamped()           # Initializes msg variable
        msg.drive.speed = self.SPEED            # Sets msg speed to entered speed
        msg.drive.acceleration = 0              # Sets msg acceleration to 0
        msg.drive.jerk = 1                      # Sets msg jerk to 1
        msg.drive.steering_angle = angle        # Sets msg steering angle to entered angle
        msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
        self.DrivePub.publish(msg)              # Publishes the message



    # Function: PIDWallFollow
    # Parameters: msg (ObjectDetection)
    #
    # This function uses a PID control system
    # to let the car follow a line

    def PIDWallFollow(self, msg):

        distance = min(msg.dists)       # Finds the minimum range
        steering_angle = 0              # Initializes steering_angle

        # SET PID PARAMETERS
        THRESHOLD = 0.05                # Sets threshold to 5cm
        Kp = 0.2
        Kd = 0.1
        Ki = 0.1

        # CALCULATE STEERING ANGLE
        e_deriv = (error - self.prev_error) / (time.clock() - self.prev_time)
        e_int = (error + self.prev_error)/2 * (time.clock() - self.prev_time)
        self.prev_error = error
        self.prev_time = time.clock()


        # PUBLISH DRIVE COMMAND
        self.drive(Kp*error + Kd*e_deriv + Ki*e_int)    # Execute drive function

