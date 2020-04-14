#ifndef PANDA_DUAL__SIMULTANEOUS_CONTROL_H
#define PANDA_DUAL__SIMULTANEOUS_CONTROL_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "panda_dual_simultaneous_control_trajectory.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

// void triggerCallback(const std_msgs::Float32::ConstPtr& msg);
void rightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void leftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void controllerCallbackProcessed(const geometry_msgs::PoseStamped::ConstPtr& msg);
void SubtractPose(geometry_msgs::Pose &pose, geometry_msgs::Pose startingPose);
void AddPose(geometry_msgs::Pose &pose, geometry_msgs::Pose originPose);
void rightMenuCallback(const std_msgs::Int32::ConstPtr& msg);
void leftMenuCallback(const std_msgs::Int32::ConstPtr& msg);
void moveRobot();
void rightTriggerCallback(const std_msgs::Float32::ConstPtr& msg);
void leftTriggerCallback(const std_msgs::Float32::ConstPtr& msg);
void rightGripCallback(const std_msgs::Int32::ConstPtr& msg);
void leftGripCallback(const std_msgs::Int32::ConstPtr& msg);

// void ControlProcessPose(RobotArm &robotArm);
void DualControlProcessPose();
// geometry_msgs::Pose ControllerProcessPose(geometry_msgs::Pose pose, RobotArm *robotArm);
// geometry_msgs::Pose ControllerProcessPose(geometry_msgs::Pose pose, RobotArm *robotArm);

class RobotArm {
    public:
        geometry_msgs::PoseStamped targetPose;
        geometry_msgs::PoseStamped finalTargetPose;
        geometry_msgs::PoseStamped VR_rawPose;
        geometry_msgs::Pose currentPoseRobot; // Used for calculation VR controller transformation
        geometry_msgs::Pose VR_startingPose; // Used for calculation VR controller transformation
        moveit_msgs::RobotTrajectory trajectory;
        std::string endLinkName;

        // States of robot
        int menu = 0;
        int grip = 0;
        float trigger = 0;
        
        // Cartesian path variables
        const double jump_threshold = 0.0; // Variable used in cartesian planning
        const double eef_step = 0.01; // Variable used in cartesian planning
        std::vector<geometry_msgs::Pose> waypoints; // The path the robot shall move through (cartesian path planning)
        moveit::planning_interface::MoveGroupInterface *move_group_p; // Should be the move_group of ONE arm
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // void PlanAndExecuteTrajectory(moveit::planning_interface::MoveGroupInterface *move_group_p);
        // This can control ONE arm at a time
        // send the movegroup of the arm you wanna plan for
        // OBSERVE most IK solvers cannot solve for the dual arm -> one arm at the time
        // Send the move_group of the individual arm NOT dual arm
        void PlanAndExecuteTrajectory(moveit::planning_interface::MoveGroupInterface *move_group_p){
            this->waypoints.clear();
            this->waypoints.push_back(this->targetPose.pose);

            char charEndLinkName[this->endLinkName.length() + 1]; 
            strcpy(charEndLinkName, this->endLinkName.c_str()); 
            
            move_group_p->computeCartesianPath(this->waypoints, this->eef_step, this->jump_threshold, this->trajectory, charEndLinkName);
            this->my_plan.trajectory_ = this->trajectory;
            move_group_p->execute(this->my_plan);
        };

        // Calculates a trajectory for the panda arm.
        // This is used to control the both arms at the same time to manually send the control message to 
        // the robot's controller. /robot_id/controller/command (Se link for more information:
        // https://answers.ros.org/question/335836/moveit-moveit_ros_move_groupmove_group-nodes-in-a-namespace/
        trajectory_msgs::JointTrajectory PlanJointTrajectory()
        {
            this->waypoints.push_back(this->targetPose.pose);

            char charEndLinkName[this->endLinkName.length() + 1]; 
            strcpy(charEndLinkName, this->endLinkName.c_str()); 
            
            move_group_p->computeCartesianPath(this->waypoints, this->eef_step, this->jump_threshold, this->trajectory, charEndLinkName);
            // To fix error with not increasing time (rather ugly solution but it works, fix if possible)
            for(int i = 0; i < this->trajectory.joint_trajectory.points.size(); i++)
            {
                // std::cout << this->trajectory.joint_trajectory.points[i].time_from_start << std::endl;
                this->trajectory.joint_trajectory.points[i].time_from_start += ros::Duration((double) i*0.001);
            }

            return this->trajectory.joint_trajectory;
        }

};

class RobotHandler {
    public:
        moveit::planning_interface::MoveGroupInterface *move_group_p;
        RobotArm left;
        RobotArm right;
        bool busy = false;
};

#endif //PANDA_DUAL__SIMULTANEOUS_CONTROL_H