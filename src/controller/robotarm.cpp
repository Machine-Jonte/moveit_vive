// Writtin by Jonathan Österberg
// For questions please contact: 
//                   Jonte_ost@live.com

// If you find this work to be useful
// please consider refering to it. 

#include "robotarm.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>     // std::cout
#include <algorithm>    // std::for_each
#include <vector>       // std::vector
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// If you are not using my VR publisher, please change these lines
// to match your publisher
void DataFlowManager::init(std::string controllerName, ros::NodeHandle &node_handle, RobotArm *robotArm_p)
{
    this->sub_pose      = node_handle.subscribe("/vive/controller/" + controllerName + "/pose", 1,         &RobotArm::VR_poseControllerCallback, robotArm_p);
    this->sub_trigger   = node_handle.subscribe("/vive/controller/" + controllerName + "/trigger", 1,      &RobotArm::VR_triggerCallback,        robotArm_p);
    this->sub_menu      = node_handle.subscribe("/vive/controller/" + controllerName + "/buttons/menu", 1, &RobotArm::VR_menuCallback,           robotArm_p);
    this->sub_grip      = node_handle.subscribe("/vive/controller/" + controllerName + "/buttons/grip", 1, &RobotArm::gripCallback,              robotArm_p);

    // The pose is published so that it can be displayed in 
    // rviz for easier debugging
    this->pub_processed = node_handle.advertise<geometry_msgs::PoseStamped>("/moveit_vive/" + controllerName + "/processed/pose", 1);
    this->pub_currentPose = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/" + controllerName + "/currentPose", 1);
}

Robot::Robot(int nr_arms)
{
    for(int i = 0; i < nr_arms; i++)
    {
        this->robotArms.push_back(RobotArm());
        this->robotArms.back().fullRobot = this;
    }
}

void RobotArm::init(ros::NodeHandle &node_handle, std::string endLinkName, std::string controllerName) 
{
    this->endLinkName = endLinkName;
    this->io.init(controllerName, node_handle, this);
}

void Robot::setPoseTargets()
{
    this->move_group_p->clearPoseTargets();
    this->move_group_p->setStartStateToCurrentState();
    for(int i = 0; i < this->robotArms.size(); i++)
    {
     this->move_group_p->setPoseTarget(this->robotArms[i].targetPose.pose, this->robotArms[i].endLinkName);   
    }
}


void printVector (std::string string) {
  std::cout << string << std::endl;
}

void Robot::setJointConstraints()
{
    moveit_msgs::JointConstraint jointConstraint;
    std::vector<std::string> jointNames = this->move_group_p->getJointNames();
    std::vector<double> jointValues = this->move_group_p->getCurrentJointValues();
    std::vector<moveit_msgs::JointConstraint> jointConstraints;
    std::cout << jointNames.size() << " ::: " << jointValues.size() << std::endl;
    for(int i = 0; i < jointNames.size(); i++)
    {
        moveit_msgs::JointConstraint current;
        current.joint_name = jointNames[i];
        current.position = jointValues[i];
        current.tolerance_above = 100;
        current.tolerance_below = 100;
        jointConstraints.push_back(current);
    }
    moveit_msgs::Constraints constraints;
    constraints.joint_constraints = jointConstraints;
    constraints.name = "joint_constraints";
    this->move_group_p->setPathConstraints(constraints);
    moveit_msgs::Constraints test = this->move_group_p->getPathConstraints();
    std::cout << test.name << std::endl;
}


// Related to processPose
// processPose will lock the target pose
// on the robots end effector until trigger 
// is pressed
void addPose(geometry_msgs::Pose &pose, geometry_msgs::Pose originPose)
{
    pose.position.x += originPose.position.x;
    pose.position.y += originPose.position.y;
    pose.position.z += originPose.position.z;
}

void subtractPose(geometry_msgs::Pose &pose, geometry_msgs::Pose startingPose)
{
    pose.position.x -= startingPose.position.x;
    pose.position.y -= startingPose.position.y;
    pose.position.z -= startingPose.position.z;
}

void RobotArm::processOrientation(geometry_msgs::PoseStamped &poseStamped)
{
    tf2::Quaternion q_processed, q_robot, q_controller, q_relative, q_startedTracking, q_controllerStartedTracking;

    // Load data to quaternions for easier manipulation
    tf2::fromMsg(this->currentArmPose.pose.orientation, q_robot);
    tf2::fromMsg(this->controllerState.pose.pose.orientation, q_controller);
    tf2::fromMsg(this->startedTrackingPose.pose.orientation, q_startedTracking);
    tf2::fromMsg(this->controllerState.startedTrackingPose.pose.orientation, q_controllerStartedTracking);

    // Calculate the difference in orientation 
    // after the trigger is pressed
    // q_relative = q_startedTracking * q_controller.inverse();
    q_relative = q_controllerStartedTracking * q_controller.inverse();

    if(orientationLock){
        tf2::Quaternion q_orientationLock;
        tf2::fromMsg(this->orientationLockPose.orientation, q_orientationLock);
        q_processed = q_orientationLock;
    } else {
        q_processed = q_relative.inverse() * q_robot;
    }
    tf2::convert(q_processed, poseStamped.pose.orientation);
}

void RobotArm::processPosition(geometry_msgs::PoseStamped &poseStamped)
{
    subtractPose(poseStamped.pose, this->controllerState.startedTrackingPose.pose);
    addPose(poseStamped.pose, this->startedTrackingPose.pose);
}

void RobotArm::processPose()
{
    // If the controller is not pressed
    // make the processed robot posed locke to robot pose
    if(!(this->controllerState.trigger > 0.99))
    {
        this->controllerState.startedTrackingPose = this->controllerState.pose;
        this->currentArmPose = this->fullRobot->move_group_p->getCurrentPose(this->endLinkName);
        this->startedTrackingPose = this->currentArmPose; 
    }

    geometry_msgs::PoseStamped poseStamped = this->controllerState.pose;
    poseStamped.header.stamp = ros::Time::now();
    this->processPosition(poseStamped);
    this->processOrientation(poseStamped);
    this->targetPose = poseStamped;
}

// Callback functions below
void RobotArm::VR_poseControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    this->controllerState.pose = *msg;
    this->processPose();
    
    // This is used for collecting data
    this->io.pub_processed.publish(this->targetPose);
    this->io.pub_currentPose.publish(this->fullRobot->move_group_p->getCurrentPose(this->endLinkName));
}

void RobotArm::VR_triggerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->controllerState.trigger = msg->data;
}

void RobotArm::VR_menuCallback(const std_msgs::Int32::ConstPtr& msg)
{
    this->controllerState.menu = msg->data;
    if(msg->data){
        this->menuStreak++;
        if(menuStreak > 20)
        {
            this->orientationLock = !this->orientationLock;
            if(this->orientationLock)
            {
                this->orientationLockPose = this->fullRobot->move_group_p->getCurrentPose(this->endLinkName).pose;
            }
            printf("Orientation Lock %i\n", this->orientationLock);
            menuStreak = 0;
        }
    } else {menuStreak = 0;}
}

void RobotArm::gripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    this->controllerState.grip = msg->data;
    if(this->controllerState.grip)
    {
        this->fullRobot->move_group_p->stop();
    }
}
