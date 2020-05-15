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
    // REMOVE IF WORK
    // std::string sub_pose_string = "/vive/controller/" + controllerName + "/pose";
    // char sub_pose_char[sub_pose_string.length() +1];
    // strcpy(sub_pose_char, sub_pose_string.c_str()); 
    // END REMOVE IF WORK

    this->sub_pose      = node_handle.subscribe("/vive/controller/" + controllerName + "/pose", 1,         &RobotArm::VR_poseControllerCallback, robotArm_p);
    this->sub_trigger   = node_handle.subscribe("/vive/controller/" + controllerName + "/trigger", 1,      &RobotArm::VR_triggerCallback,        robotArm_p);
    this->sub_menu      = node_handle.subscribe("/vive/controller/" + controllerName + "/buttons/menu", 1, &RobotArm::VR_menuCallback,           robotArm_p);
    this->sub_grip      = node_handle.subscribe("/vive/controller/" + controllerName + "/buttons/grip", 1, &RobotArm::gripCallback,              robotArm_p);

    // The pose is published so that it can be displayed in 
    // rviz for easier debugging
    this->pub_processed = node_handle.advertise<geometry_msgs::PoseStamped>("/moveit_vive/" + controllerName + "/processed/pose", 1);
    this->pub_currentPose = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/" + controllerName + "/currentPose", 1);
}

Robot::Robot()
{
    this->right.fullRobot = this;
    this->left.fullRobot = this;
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
    this->move_group_p->setPoseTarget(this->right.targetPose.pose, this->right.endLinkName);
    this->move_group_p->setPoseTarget(this->left.targetPose.pose, this->left.endLinkName);
}


void printVector (std::string string) {  // function:
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
    // for_each(jointNames.begin(), jointNames.end(), printVector); 
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
        q_processed = q_robot;
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
        this->orientationLock = !this->orientationLock;
    }
}

void RobotArm::gripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    this->controllerState.grip = msg->data;
    if(this->controllerState.grip)
    {
        this->fullRobot->move_group_p->stop();
    }
}



// Constraints
void Robot::setPathConstraints()
{
    moveit_msgs::Constraints constraints;
    
    moveit_msgs::PositionConstraint right = this->right.createConstraint();
    moveit_msgs::PositionConstraint left = this->left.createConstraint();

    constraints.position_constraints.push_back(right);
    constraints.position_constraints.push_back(left);
    this->move_group_p->setPathConstraints(constraints);
}

moveit_msgs::PositionConstraint RobotArm::createConstraint()
{
    geometry_msgs::Pose fromPose, toPose;
    fromPose = this->currentArmPose.pose;
    toPose = this->targetPose.pose;

    moveit_msgs::PositionConstraint positionConstraint;
    positionConstraint.header.frame_id = this->endLinkName;
    positionConstraint.link_name = this->endLinkName;
    positionConstraint.weight = 1.0;
    // positionConstraint.header.frame_id = this->fullRobot->move_group_p->getPlanningFrame();

    // c and k can be used to allow bigger planning area
    // for the robot
    double c = 2.0;
    double k = 0.3;
    double dx = ( ( abs(toPose.position.x - fromPose.position.x) ) + k ) * c;
    double dy = ( ( abs(toPose.position.y - fromPose.position.y) ) + k ) * c;
    double dz = ( ( abs(toPose.position.z - fromPose.position.z) ) + k ) * c;
    // double dx, dy, dz;
    // dx = 0.5;
    // dy = 0.5;
    // dz = 0.5;

    geometry_msgs::Pose averagePose;
    averagePose.orientation.w = 1.0;
    averagePose.position.x = (toPose.position.x - fromPose.position.x)/2;
    averagePose.position.y = (toPose.position.y - fromPose.position.y)/2;
    averagePose.position.z = (toPose.position.z - fromPose.position.z)/2;
    // averagePose.position.x = (toPose.position.x + fromPose.position.x)/2.0;
    // averagePose.position.y = (toPose.position.y + fromPose.position.y)/2.0;
    // averagePose.position.z = (toPose.position.z + fromPose.position.z)/2.0;



    moveit_msgs::BoundingVolume boundingVolume = this->createBoundingVolume({dx,dy,dz}, averagePose);
    
    positionConstraint.constraint_region = boundingVolume;
    geometry_msgs::Vector3 vector;
    vector.x = toPose.position.x - fromPose.position.x;
    vector.y = toPose.position.y - fromPose.position.y; 
    vector.z = toPose.position.z - fromPose.position.z;
    positionConstraint.target_point_offset = vector;

    return positionConstraint;
}
 
moveit_msgs::BoundingVolume RobotArm::createBoundingVolume(std::vector<double> size, geometry_msgs::Pose boundingBoxPose)
{
    moveit_msgs::BoundingVolume boundingVolume;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = size[0]; // x
    primitive.dimensions[1] = size[1]; // y
    primitive.dimensions[2] = size[2]; // z

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = boundingBoxPose.orientation.w;
    box_pose.orientation.x = boundingBoxPose.orientation.x;
    box_pose.orientation.y = boundingBoxPose.orientation.y;
    box_pose.orientation.z = boundingBoxPose.orientation.z;
    box_pose.position.x = boundingBoxPose.position.x;
    box_pose.position.y = boundingBoxPose.position.y;
    box_pose.position.z = boundingBoxPose.position.z;

    boundingVolume.primitives.push_back(primitive);
    boundingVolume.primitive_poses.push_back(box_pose);


    return boundingVolume;
}
