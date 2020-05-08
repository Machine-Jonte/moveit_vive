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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
// #include <visualization_msgs.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PositionConstraint.h>

#include "ros/ros.h"

#include "moveit_workspace.h"

#include <geometry_msgs/Vector3.h>
#include <moveit_msgs/BoundingVolume.h>

void rightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void leftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void controllerCallbackProcessed(const geometry_msgs::PoseStamped::ConstPtr& msg);
void SubtractPose(geometry_msgs::Pose &pose, geometry_msgs::Pose startingPose);
void AddPose(geometry_msgs::Pose &pose, geometry_msgs::Pose originPose);
void rightMenuCallback(const std_msgs::Int32::ConstPtr& msg);
void leftMenuCallback(const std_msgs::Int32::ConstPtr& msg);
void rightTriggerCallback(const std_msgs::Float32::ConstPtr& msg);
void leftTriggerCallback(const std_msgs::Float32::ConstPtr& msg);
void rightGripCallback(const std_msgs::Int32::ConstPtr& msg);
void leftGripCallback(const std_msgs::Int32::ConstPtr& msg);
geometry_msgs::PoseStamped copyPose(geometry_msgs::PoseStamped poseStamped);
void DualControlProcessPose();
double euclideanDistancePose(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);


moveit_msgs::BoundingVolume createBoundingVolume(std::vector<double> size, geometry_msgs::Pose boundingBoxPose);

class RobotArm {
    public:
        geometry_msgs::PoseStamped targetPose;
        geometry_msgs::PoseStamped finalTargetPose;
        geometry_msgs::PoseStamped VR_rawPose;
        geometry_msgs::PoseStamped currentPoseRobot; // Used for calculation VR controller transformation
        geometry_msgs::PoseStamped basePose; // Used for calculation VR controller transformation
        geometry_msgs::Pose VR_startingPose; // Used for calculation VR controller transformation
        moveit_msgs::RobotTrajectory trajectory;
        std::string endLinkName;
        ros::Publisher visualization_pub;
        std::vector<geometry_msgs::Pose> waypoints_draw; // The path the robot shall move through (cartesian path planning)
        ros::Publisher currentPose_pub;
        ros::Publisher targetPose_pub;
        ros::Publisher controller_pub;
        double executionTimeEnd = 0;
        MoveItWorkSpace *workspace_p;

        // std::vector<double> jointValues;

        // States of robot
        int menu = 0;
        int grip = 0;
        float trigger = 0;
        
        // Cartesian path variables
        const double jump_threshold = 0.0; // Variable used in cartesian planning
        const double eef_step = 0.01; // Variable used in cartesian planning
        std::vector<geometry_msgs::Pose> waypoints; // The path the robot shall move through (cartesian path planning)
        std::vector<geometry_msgs::Pose> waypointsWaiting; // The path the robot shall move through (cartesian path planning)
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
        }

        // Calculates a trajectory for the panda arm.
        // This is used to control the both arms at the same time to manually send the control message to 
        // the robot's controller. /robot_id/controller/command (Se link for more information:
        // https://answers.ros.org/question/335836/moveit-moveit_ros_move_groupmove_group-nodes-in-a-namespace/
        double PlanJointTrajectory()
        {
            geometry_msgs::Pose fromPose, toPose;
            fromPose = this->currentPoseRobot.pose;
            toPose = this->waypoints.back();

            // this->workspace_p->init(fromPose, toPose, this->move_group_p->getPlanningFrame(), this->endLinkName);

            // this->workspace_p->calculateWorkspace();
            // this->workspace_p->add();




            moveit_msgs::Constraints constraints;
            moveit_msgs::PositionConstraint positionConstraint;
            

            double c = 1.0;
            double k = 0.1;
            double dx = ( ( abs(toPose.position.x - fromPose.position.x) ) + k ) * c;
            double dy = ( ( abs(toPose.position.y - fromPose.position.y) ) + k ) * c;
            double dz = ( ( abs(toPose.position.z - fromPose.position.z) ) + k ) * c;

        

            positionConstraint.link_name = this->endLinkName;

            geometry_msgs::Pose averagePose;
            averagePose.orientation.w = 1.0;
            // averagePose.position.x = (toPose.position.x + fromPose.position.x)/2.0;
            // averagePose.position.y = (toPose.position.y + fromPose.position.y)/2.0;
            // averagePose.position.z = (toPose.position.z + fromPose.position.z)/2.0;

            averagePose.position.x = 0.0;
            averagePose.position.y = 0.0;
            averagePose.position.z = 0.0;


            moveit_msgs::BoundingVolume boundingVolume = createBoundingVolume({dx,dy,dz}, averagePose);
            positionConstraint.constraint_region = boundingVolume;
            positionConstraint.header.frame_id = this->endLinkName;
            positionConstraint.weight = 1.0;
            // positionConstraint.constraint_region

            // geometry_msgs::Vector3 target_point_offset;
            // target_point_offset.x = dx;
            // target_point_offset.y = dy;
            // target_point_offset.z = dz;

            // positionConstraint.target_point_offset;

            constraints.position_constraints.push_back(positionConstraint);



            char charEndLinkName[this->endLinkName.length() + 1]; 
            strcpy(charEndLinkName, this->endLinkName.c_str()); 

            double success = move_group_p->computeCartesianPath(this->waypoints, this->eef_step, this->jump_threshold, this->trajectory, constraints, charEndLinkName);
            // double success = move_group_p->computeCartesianPath(this->waypoints, this->eef_step, this->jump_threshold, this->trajectory, charEndLinkName);
            // To fix error with not increasing time (rather ugly solution but it works, fix if possible)
            for(int i = 0; i < this->trajectory.joint_trajectory.points.size(); i++)
            {
                // this->trajectory.joint_trajectory.header.stamp = ros::Time::now();
                // std::cout << this->trajectory.joint_trajectory.points[i].time_from_start << std::endl;
                this->trajectory.joint_trajectory.points[i].time_from_start += ros::Duration((double) i*0.001);
            }

            // this->workspace_p->remove();

            // return this->trajectory.joint_trajectory;
            return success;
        }

};

class RobotHandler {
    public:
        moveit::planning_interface::MoveGroupInterface *move_group_p;
        RobotArm left;
        RobotArm right;
        bool busy = false;
};


class PublishHandlerRobotState
{
    public:
        ros::Publisher currentState; // End effector publisher. Send poseStamped and in header set the arm_id as frame_id
        ros::Publisher jointValues; // All joint values as a multidimensional array of float msg
        ros::Publisher targetPose; // End effectors wanted pose
        // ros::Publisher 
};


void moveRobot(RobotArm &robotArm);
void onlineUpdate(RobotArm &robotArm);
void ControlProcessPose(RobotArm &robotArm);
void PublishVisualization(RobotArm robotArm);
void moveRobot(RobotArm &robotArm);

#endif //PANDA_DUAL__SIMULTANEOUS_CONTROL_H