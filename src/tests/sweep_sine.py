#!/usr/bin/env python3
# publisher node
"""
A simple script that sends fake controller position and orientation.
This is used to investigate system properties.
"""
import rospy
import numpy as np
from math import sin, radians, pi
import csv

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32, Float32

class SweepSine:
    def __init__(self, lower_freq, upper_freq, period_time):
        self.f1 = upper_freq
        self.f0 = lower_freq
        self.T = period_time
    
    def calc(self,t):
        # Linear sweep sine
        return sin(2*pi* (self.f0*t+(self.f1-self.f0)/(self.T)*pow(t,2)))

        # Exponential sweep sine
        # return sin(2*pi*self._freq(t) * t)

    def _freq(self, t):
        f0 = 0.0
        f1 = 10.0
        return (self.T*((pow(f1/(f0+0.1),t/self.T)-1)/np.log(f1/(f0+0.1)))/t-1)/2.0
 

if __name__ == "__main__":
    rospy.init_node("fake_controller_commands")

    pub_pose = [rospy.Publisher("/vive/controller/left/pose", PoseStamped, queue_size=1),
                rospy.Publisher("/vive/controller/right/pose", PoseStamped, queue_size=1) ]

    pub_enabler = [rospy.Publisher("/vive/controller/left/trigger", Float32, queue_size=1), 
                   rospy.Publisher("/vive/controller/right/trigger", Float32, queue_size=1)]

    poses = [PoseStamped(), PoseStamped()]

    max_time = 400.0
    sweepsine = SweepSine(0, 0.5, max_time)
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
                pub_enabler[i].publish(0)
                pub.publish(poses[i])
        else:

            
            for pub in pub_enabler:
                pub.publish(1)
            
            for i, pub in enumerate(pub_pose):
                poses[i].header.stamp = rospy.Time.now()
                poses[i].pose.position.y = sweepsine.calc(t) / 5            
                pub.publish(poses[i])

            # print("Pose y:", poses[0].pose.position.y)
        print("Elapsed Time:" , t)
        t += 1.0/freq
        if t > max_time:
            exit()
        rate.sleep()