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
#include "ros/ros.h"



class Robot;
class RobotArm;


class ControllerState {
    public:
        geometry_msgs::PoseStamped pose;
        float trigger = 0;
        int menu = 0;
        int grip = 0;

        // Related to ProcessPose()
        geometry_msgs::PoseStamped startedTrackingPose;
};

class DataFlowManager {
    public:
        ros::Subscriber sub_pose;
        ros::Subscriber sub_trigger; 
        ros::Subscriber sub_menu;
        ros::Subscriber sub_grip;
        ros::Publisher  pub_processed;

        void init(std::string controllerName, ros::NodeHandle &node_handle, RobotArm *robotArm_p);
};

class RobotArm {
    public:
        std::string endLinkName;
        ControllerState controllerState;
        DataFlowManager io;
        geometry_msgs::PoseStamped targetPose;
        geometry_msgs::PoseStamped startedTrackingPose;
        moveit::planning_interface::MoveGroupInterface *move_group_p;
        
        Robot *fullRobot;

        void init(ros::NodeHandle &node_handle, std::string endLinkName, std::string controllerName);

        void ProcessPose();

        void VR_PoseControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);      
        void VR_TriggerCallback(const std_msgs::Float32::ConstPtr& msg);
        void VR_MenuCallback(const std_msgs::Int32::ConstPtr& msg);
        void GripCallback(const std_msgs::Int32::ConstPtr& msg);

};

class Robot {
    public: 
        RobotArm right;
        RobotArm left;
        moveit::planning_interface::MoveGroupInterface *move_group_p;

        void setPoseTargets();

        Robot();

};



#endif 