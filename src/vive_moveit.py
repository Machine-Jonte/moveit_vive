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

# Init parameters
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_vive', anonymous=True)

class RobotHandler:
  def __init__(self):
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "panda_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    self.initPosition()
    self.starting_pose = self.move_group.get_current_pose()

  def initPosition(self):
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0



    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

  def setPose(self, w, x, y, z):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = w
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    self.move_group.set_pose_target(pose_goal)

    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

  def setPoseFromPose(self, pose):
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = pose.position.x
    # pose_goal.position.y = pose.position.y
    # pose_goal.position.z = pose.position.z
    print("Target Pose:")
    print("x", pose.position.x)
    print("y", pose.position.y)
    print("z", pose.position.z)

    print("Starting Pose:")
    print("x", self.starting_pose.pose.position.x)
    print("y", self.starting_pose.pose.position.y)
    print("z", self.starting_pose.pose.position.z)


    # pose.position.x = self.starting_pose.pose.position.x + pose.position.x
    # pose.position.y = self.starting_pose.pose.position.y + pose.position.y
    # pose.position.z = self.starting_pose.pose.position.z + pose.position.z
    

    self.move_group.set_pose_target(pose)

    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()


robotHandler = RobotHandler()

def main():
  print("Starting the node moveit_vive")
  # robotHandler.initPosition()
  # robotHandler.setPose(1.0, 0.4, 0.1, 0.9)
  # robotHandler.setPose(0.0, 0.4, 0.1, 0.9)

  listener()



def listener():
    rospy.Subscriber("/vive/pose2", PoseStamped, callback, queue_size=1)
    rospy.spin()




def callback(msg):
  print("Got message")
  robotHandler.setPoseFromPose(msg.pose)
  



if __name__ == "__main__":
  main()