#!/usr/bin/env python
"""
A script that shows the trajectory taken by the robot system
"""

from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
import rospy
import os

class Trajectory:
    def __init__(self):
        self.left  = []
        self.right = []

        self.sub = [rospy.Subscriber("/robot/right/currentPose", PoseStamped, self.rightCallback, queue_size=1), 
                    rospy.Subscriber("/robot/left/currentPose",  PoseStamped, self.leftCallback, queue_size=1)]
                
        self.sub_grip = [rospy.Subscriber("/vive/controller/right/buttons/grip", Int32, self.gripCallback, queue_size=1), 
                         rospy.Subscriber("/vive/controller/left/buttons/grip",  Int32, self.gripCallback, queue_size=1)]

    def gripCallback(self, msg):
        if msg.data == 1:
            self.left  = []
            self.right = []

    def leftCallback(self, msg):
        self.left.append(msg)
        if len(self.left) > 1000:
            self.left = self.left[len(self.left)-1000:]
    
    def rightCallback(self, msg):
        self.right.append(msg)
        if len(self.right) > 1000:
            self.right = self.right[len(self.right)-1000:]


    def publish_visualization(self, poseArray, name):
        pub = rospy.Publisher("/moveit_vive/visualization/" + name + "/trajectory", Marker, queue_size=1)
        points = Marker()
        line_strip = Marker()

        points.header.frame_id = line_strip.header.frame_id = "/world"
        points.header.stamp = line_strip.header.stamp = rospy.Time.now()
        points.ns = line_strip.ns = "points_and_lines"
        points.action = points.ADD
        line_strip.action = line_strip.ADD
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0


        points.id = 0
        line_strip.id = 1
    
        points.type = points.POINTS
        line_strip.type = line_strip.LINE_STRIP

        # // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05
        points.scale.y = 0.05
        # // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.01
        # // Points are green
        points.color.g = 1.0
        points.color.a = 1.0
        # // Line strip is blue
        line_strip.color.b = 1.0
        line_strip.color.a = 1.0

        for i in range(len(poseArray)):
            p = Point()
            p.x = poseArray[i].pose.position.x
            p.y = poseArray[i].pose.position.y
            p.z = poseArray[i].pose.position.z

            points.points.append(p)
            line_strip.points.append(p)

        # pub.publish(points)
        pub.publish(line_strip)

if __name__ == "__main__":
    rospy.init_node("trajectory_visualization")
    trajectory = Trajectory()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        trajectory.publish_visualization(trajectory.left, "left")
        trajectory.publish_visualization(trajectory.right, "right")
        rate.sleep()