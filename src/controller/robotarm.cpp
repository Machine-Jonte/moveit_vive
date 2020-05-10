// Writtin by Jonathan Österberg
// For questions please contact: 
//                   Jonte_ost@live.com

// If you find this work to be useful
// please consider refering to it. 

#include "robotarm.h"

// If you are not using my VR publisher, please change these lines
// to match your publisher
void DataFlowManager::init(std::string controllerName, ros::NodeHandle &node_handle, RobotArm *robotArm_p)
{
    // REMOVE IF WORK
    // std::string sub_pose_string = "/vive/controller/" + controllerName + "/pose";
    // char sub_pose_char[sub_pose_string.length() +1];
    // strcpy(sub_pose_char, sub_pose_string.c_str()); 
    // END REMOVE IF WORK

    this->sub_pose      = node_handle.subscribe("/vive/controller/" + controllerName + "/pose", 1,         &RobotArm::VR_PoseControllerCallback, robotArm_p);
    this->sub_trigger   = node_handle.subscribe("/vive/controller/" + controllerName + "/trigger", 1,      &RobotArm::VR_TriggerCallback,        robotArm_p);
    this->sub_menu      = node_handle.subscribe("/vive/controller/" + controllerName + "/buttons/menu", 1, &RobotArm::VR_MenuCallback,           robotArm_p);
    this->sub_grip      = node_handle.subscribe("/vive/controller/" + controllerName + "/buttons/grip", 1, &RobotArm::GripCallback,              robotArm_p);

    // The pose is published so that it can be displayed in 
    // rviz for easier debugging
    this->pub_processed = node_handle.advertise<geometry_msgs::PoseStamped>("/moveit_vive/" + controllerName + "/processed/pose", 1);
}

Robot::Robot()
{
    this->right.fullRobot = this;
    this->left.fullRobot = this;
}

void Robot::setPoseTargets()
{
    this->move_group_p->setPoseTarget(this->right.targetPose.pose, this->right.endLinkName);
    this->move_group_p->setPoseTarget(this->left.targetPose.pose, this->left.endLinkName);
}

void RobotArm::init(ros::NodeHandle &node_handle, std::string endLinkName, std::string controllerName) 
{
    this->endLinkName = endLinkName;
    this->io.init(controllerName, node_handle, this);
}

// Related to ProcessPose
// ProcessPose will lock the target pose
// on the robots end effector until trigger 
// is pressed
void AddPose(geometry_msgs::Pose &pose, geometry_msgs::Pose originPose)
{
    pose.position.x += originPose.position.x;
    pose.position.y += originPose.position.y;
    pose.position.z += originPose.position.z;
}

void SubtractPose(geometry_msgs::Pose &pose, geometry_msgs::Pose startingPose)
{
    pose.position.x -= startingPose.position.x;
    pose.position.y -= startingPose.position.y;
    pose.position.z -= startingPose.position.z;
}

void RobotArm::ProcessPose()
{
    if(!(this->controllerState.trigger > 0.99))
    {
        this->controllerState.startedTrackingPose = this->controllerState.pose;
        this->startedTrackingPose = this->fullRobot->move_group_p->getCurrentPose(this->endLinkName);
    }

    geometry_msgs::PoseStamped poseStamped = this->controllerState.pose;
    SubtractPose(poseStamped.pose, this->controllerState.startedTrackingPose.pose);
    AddPose(poseStamped.pose, this->startedTrackingPose.pose);
    poseStamped.header.stamp = ros::Time::now();
    this->targetPose = poseStamped;
    this->io.pub_processed.publish(this->targetPose);
}

// Callback functions below
void RobotArm::VR_PoseControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    this->controllerState.pose = *msg;
    this->ProcessPose();
}

void RobotArm::VR_TriggerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->controllerState.trigger = msg->data;
}

void RobotArm::VR_MenuCallback(const std_msgs::Int32::ConstPtr& msg)
{
    this->controllerState.menu = msg->data;
}

void RobotArm::GripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    this->controllerState.grip = msg->data;
}

 