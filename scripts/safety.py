#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from racecar_wk3.msg import ObjectDetections


class SafetyController:
    def __init__(self):
        self.safety_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)
        self.objects_sub = rospy.Subscriber("objects", ObjectDetections, self.objects_cb)

    def objects_cb(self, msg):
        for o in range(len(msg.dists)):
            if msg.dists[o] < .5 and 120 < msg.angles[o] and msg.angles[o] < 150:
                stop = AckermannDriveStamped()
                stop.header.stamp = rospy.Time.now()
                stop.drive.speed = -1
                r = rospy.Rate(6)
                for i in range(6):
                    self.safety_pub.publish(stop)
                    r.sleep()


if __name__ == "__main__":
    rospy.init_node("safety")
    sc = SafetyController()
    rospy.spin()
