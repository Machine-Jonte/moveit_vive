#!/usr/bin/env python
"""
A simple script that sends fake controller position and orientation.
This is used to investigate system properties.
"""
import rospy
import numpy
from math import sin, radians, pi
import csv

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32

class SweepSine:
    def __init__(self, lower_freq, upper_freq, period_time):
        self.f1 = upper_freq
        self.f0 = lower_freq
        self.T = period_time
    
    def calc(self,t):
        return sin(2*pi* (self.f0*t+(self.f1-self.f0)/(2*self.T)*pow(t,2)))
 

if __name__ == "__main__":
    rospy.init_node("fake_controller_commands")

    pub_pose = [rospy.Publisher("/vive/controller/left/pose", PoseStamped, queue_size=1),
                rospy.Publisher("/vive/controller/right/pose", PoseStamped, queue_size=1) ]

    pub_menu = [rospy.Publisher("/vive/controller/left/buttons/menu", Int32, queue_size=1), 
                rospy.Publisher("/vive/controller/right/buttons/menu", Int32, queue_size=1)]

    poses = [PoseStamped(), PoseStamped()]

    sweepsine = SweepSine(0.00001, 1, 500.0)
    freq = 100.0
    rate = rospy.Rate(freq)
    rounds = 0
    t = 0
    while not rospy.is_shutdown():
        rounds += 1
        if rounds < freq*2:
            for pose in poses:
                pose.header.frame_id = "base"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = 0
                pose.pose.position.y = 0
                pose.pose.position.z = 0

            for i, pub in enumerate(pub_pose):
                pub_menu[i].publish(0)
                pub.publish(poses[i])
        else:

            
            for pub in pub_menu:
                pub.publish(1)
            
            for i, pub in enumerate(pub_pose):
                poses[i].header.stamp = rospy.Time.now()
                poses[i].pose.position.y = sweepsine.calc(t) / 5            
                pub.publish(poses[i])

            print("Pose y:", poses[0].pose.position.y)
        t += 1.0/freq
        rate.sleep()