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
    this->move_group_p->setPoseTarget(this->right.targetPose.pose, this->right.endLinkName);
    this->move_group_p->setPoseTarget(this->left.targetPose.pose, this->left.endLinkName);
}

void Robot::setPathConstraints()
{
    std::cout << "Entering here" << std::endl;
    moveit_msgs::Constraints constraints;
    moveit_msgs::PositionConstraint right = this->right.createConstraint();
    moveit_msgs::PositionConstraint left = this->left.createConstraint();
    constraints.position_constraints.push_back(right);
    constraints.position_constraints.push_back(left);
    this->move_group_p->setPathConstraints(constraints);
    // this->move_group_p->setPathConstraints(left);
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

void RobotArm::processPose()
{
    if(!(this->controllerState.trigger > 0.99))
    {
        this->controllerState.startedTrackingPose = this->controllerState.pose;
        this->currentArmPose = this->fullRobot->move_group_p->getCurrentPose(this->endLinkName);
        this->startedTrackingPose = this->currentArmPose; 
    }

    geometry_msgs::PoseStamped poseStamped = this->controllerState.pose;
    subtractPose(poseStamped.pose, this->controllerState.startedTrackingPose.pose);
    addPose(poseStamped.pose, this->startedTrackingPose.pose);
    poseStamped.header.stamp = ros::Time::now();
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
}

void RobotArm::gripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    this->controllerState.grip = msg->data;
    if(this->controllerState.grip)
    {
        this->fullRobot->move_group_p->stop();
    }
}

moveit_msgs::PositionConstraint RobotArm::createConstraint()
{
    geometry_msgs::Pose fromPose, toPose;
    fromPose = this->currentArmPose.pose;
    toPose = this->targetPose.pose;

    moveit_msgs::PositionConstraint positionConstraint;
    positionConstraint.link_name = this->endLinkName;
    positionConstraint.weight = 1.0;

    // c and k can be used to allow bigger planning area
    // for the robot
    double c = 1.0;
    double k = 0.1;
    double dx = ( ( abs(toPose.position.x - fromPose.position.x) ) + k ) * c;
    double dy = ( ( abs(toPose.position.y - fromPose.position.y) ) + k ) * c;
    double dz = ( ( abs(toPose.position.z - fromPose.position.z) ) + k ) * c;

    geometry_msgs::Pose averagePose;
    averagePose.orientation.w = 1.0;
    averagePose.position.x = 0.0;
    averagePose.position.y = 0.0;
    averagePose.position.z = 0.0;
    // averagePose.position.x = (toPose.position.x + fromPose.position.x)/2.0;
    // averagePose.position.y = (toPose.position.y + fromPose.position.y)/2.0;
    // averagePose.position.z = (toPose.position.z + fromPose.position.z)/2.0;



    moveit_msgs::BoundingVolume boundingVolume = this->createBoundingVolume({dx,dy,dz}, averagePose);
    
    positionConstraint.constraint_region = boundingVolume;
    positionConstraint.header.frame_id = this->endLinkName;
    // std::cout << this->fullRobot->move_group_p->getPlanningFrame() << std::endl;
    // positionConstraint.header.frame_id = this->fullRobot->move_group_p->getPlanningFrame();

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
    box_pose.orientation.w = 1.0;
    box_pose.position.x = boundingBoxPose.position.x;
    box_pose.position.y = boundingBoxPose.position.y;
    box_pose.position.z = boundingBoxPose.position.z;

    boundingVolume.primitives.push_back(primitive);
    boundingVolume.primitive_poses.push_back(box_pose);
    // boundingVolume.operation = boundingVolume.ADD;


    return boundingVolume;
}