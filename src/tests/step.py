#!/usr/bin/env python3
# publisher node
"""
A simple script that sends fake controller position and orientation.
This is used to investigate system properties.
"""
import rospy
import numpy
from math import sin, radians, pi
import csv

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32, Float32

class StepWave:
    def __init__(self, period_time):
        self.T = float(period_time)
        self.freq = 1.0/period_time
    
    def calc(self,t):
        # return sin(2*pi* (self.f0*t+(self.f1-self.f0)/(2*self.T)*pow(t,2)))
        f = sin(2*pi*self.freq*t)
        print(f)
        if f > 0:
            return 1
        else:
            return -1
 
def getAmplitude(t, period_time):
    amplitude_base = 0.01
    return int(t/period_time) * amplitude_base
    

if __name__ == "__main__":
    rospy.init_node("fake_controller_commands")

    pub_pose = [rospy.Publisher("/vive/controller/left/pose", PoseStamped, queue_size=1),
                rospy.Publisher("/vive/controller/right/pose", PoseStamped, queue_size=1) ]

    pub_enabler = [rospy.Publisher("/vive/controller/left/trigger", Float32, queue_size=1), 
                  rospy.Publisher("/vive/controller/right/trigger", Float32, queue_size=1)]

    poses = [PoseStamped(), PoseStamped()]

    stepwave = StepWave(10.0)
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
            # Pretend menu button is pressed => the robot will track the pose
            for pub in pub_enabler:
                pub.publish(1)
            
            for i, pub in enumerate(pub_pose):
                poses[i].header.stamp = rospy.Time.now()
                # Line to change!!! (if change direction)
                poses[i].pose.position.y = stepwave.calc(t) * getAmplitude(t, stepwave.T)            
                pub.publish(poses[i])

            print("Pose y:", poses[0].pose.position.y)
        t += 1.0/freq
        rate.sleep()