// Writtin by Jonathan Österberg
// For questions please contact: 
//                   Jonte_ost@live.com

// If you find this work to be useful
// please consider refering to it. 

#ifndef ROBOTARM_H
#define ROBOTARM_H

#include "robotarm.h"

// Standard import
#include <string>
#include <vector>

// Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

// MoveIt and ROS
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/JointConstraint.h>
#include "ros/ros.h"



class Robot;
class RobotArm;


class ControllerState {
    public:
        geometry_msgs::PoseStamped pose;
        float trigger = 0;
        int menu = 0;
        int grip = 0;

        // Related to processPose()
        geometry_msgs::PoseStamped startedTrackingPose;
};

class DataFlowManager {
    public:
        ros::Subscriber sub_pose;
        ros::Subscriber sub_trigger; 
        ros::Subscriber sub_menu;
        ros::Subscriber sub_grip;
        ros::Publisher  pub_processed;
        ros::Publisher  pub_currentPose;

        void init(std::string controllerName, ros::NodeHandle &node_handle, RobotArm *robotArm_p);
};

class RobotArm {
    public:
        std::string endLinkName;
        ControllerState controllerState;
        DataFlowManager io;
        geometry_msgs::PoseStamped targetPose;
        geometry_msgs::PoseStamped startedTrackingPose;
        geometry_msgs::PoseStamped currentArmPose;
        geometry_msgs::Pose orientationLockPose;
        
        bool orientationLock = false;
        int menuStreak = 0;
        Robot *fullRobot;

        void init(ros::NodeHandle &node_handle, std::string endLinkName, std::string controllerName);

        void processPose();
        void processOrientation(geometry_msgs::PoseStamped &poseStamped);
        void processPosition(geometry_msgs::PoseStamped &poseStamped);

        void VR_poseControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);      
        void VR_triggerCallback(const std_msgs::Float32::ConstPtr& msg);
        void VR_menuCallback(const std_msgs::Int32::ConstPtr& msg);
        void gripCallback(const std_msgs::Int32::ConstPtr& msg);
        moveit_msgs::PositionConstraint createConstraint();

    private:
        moveit_msgs::BoundingVolume createBoundingVolume(std::vector<double> size, geometry_msgs::Pose boundingBoxPose);
};

class Robot {
    public:
        // RobotArm right;
        // RobotArm left;
        std::vector<RobotArm> robotArms;
        moveit::planning_interface::MoveGroupInterface *move_group_p;

        void setPoseTargets();
        void setPathConstraints();
        void setJointConstraints();

        Robot(int nr_arms);

};



#endif 