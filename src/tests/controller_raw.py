#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from geometry_msgs.msg import PoseStamped

class DataHolder:
    def __init__(self):
        self.left = PoseStamped()
        self.right = PoseStamped()

        self.sub = [rospy.Subscriber("/vive/controller/right/pose", PoseStamped, self.rightCallback, queue_size=1), 
                    rospy.Subscriber("/vive/controller/left/pose",  PoseStamped, self.leftCallback, queue_size=1)]


    def leftCallback(self, msg):
        self.left = msg
    
    def rightCallback(self, msg):
        self.right = msg



if __name__=="__main__":
    # Init parameters
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('controller_raw', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "dual"
    group = moveit_commander.MoveGroupCommander(group_name)
    right_end_link = "panda_1_link8"
    left_end_link  = "panda_2_link8"

    dataholder = DataHolder()

    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                            moveit_msgs.msg.DisplayTrajectory,
    #                                            queue_size=20)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        group.set_pose_target(dataholder.right, right_end_link)
        group.set_pose_target(dataholder.left,  left_end_link )
        group.go()
        group.stop()
        group.clear_pose_targets()
        rate.sleep()
    